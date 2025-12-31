import threading
import time

import grpc
import numpy as np
import torch
from google.protobuf import json_format
from loguru import logger
from pydantic import BaseModel
from python_client import imu_packet_pb2, imu_packet_pb2_grpc
from statemachine import StateMachine, State
from scipy.spatial.transform import Rotation as R

from modules.constants import trivial_mano_pose, trivial_mano_tensor, trivial_position
from modules.datamodels import ImuMsg, MoCapHandConfig
from modules.enumerates import MoCapHandStateEnum, MoCapCalibrateRootType, MoCapCalibrateInstallationType, \
    MoCapCalibrateShapeType, MoCapCalibrateWristType, \
    MoCapHandIDEnum, MoCapCalibrateShapeKeyPointType, MoCapThreadPluginEnum
from modules.hand.calibration import Raw2MANOCalibrator, CalibrationQuatSet, WristPoseCalibrator, point_match, \
    keypoints_verts_ids
from modules.mano_model import ManoState
from modules.ring_buffer import RingBuffer
from modules.utils import convert_frame_to_quaterion_array_in_mano_order, get_optimized_shape_params, \
    get_optimized_shape_params_from_keypoints
from modules.dataflow import get_averaged_hand_pose_input, get_seq_hand_pose_input, get_averaged_tracker_position_input, \
    get_averaged_tracker_rotation_input


class HandStateMachineFlags(BaseModel):
    compute_mano: bool = False
    shutdown: bool = False


def hand_grpc_receiver(
        kill_switch: threading.Event,
        stub: imu_packet_pb2_grpc.IMUPacketServiceStub,
        buffer: RingBuffer,
):
    start_t = time.time()
    desire_cps = 30
    n_frame = 0
    n_call = 0

    diag_t = start_t
    diag_n_frame = n_frame

    response = stub.GetPacketArrayStream(imu_packet_pb2.IMUPacketRequest(timestamp=time.time_ns()))

    while True:
        if kill_switch.is_set():
            break

        # limit the rate
        if (time.time() - start_t) * desire_cps <= n_call:
            time.sleep(0.01)
            continue

        try:
            response._deadline = time.time() + 0.1
            # Use select to implement a read timeout for gRPC stream responses.
            raw_data = next(response)
        except Exception as e:
            logger.exception(f"imu grpc error: {e}")
            return

        pose_frames = [json_format.MessageToDict(x)['packets'] for x in raw_data.packets]
        time_frames = [int(x.sys_ticks / 1000) for x in raw_data.packets]  # convert ns to us
        # frame = [json_format.MessageToDict(raw_data)['packets']]
        try:
            quat_nps: list[np.ndarray] = [convert_frame_to_quaterion_array_in_mano_order(frame) for frame in
                                          pose_frames]
        except KeyError as e:
            logger.error(f"KeyError encountered: {e}, consider reconfigure the IMU")
            continue

        # handle mano calibration
        [
            buffer.push(
                ImuMsg(sys_ticks=t, imu_rotation=R.from_quat(imu_quat), seq=n_frame + i)
            ) for i, (t, imu_quat) in enumerate(zip(time_frames, quat_nps))
        ]
        # loop update
        n_frame += len(pose_frames)
        n_call += 1

        if (time.time() - diag_t) >= 10:
            logger.info(
                f"imu grpc fps: {(n_frame - diag_n_frame) / (time.time() - diag_t)}, delay is {int(time.time() * 1000000 - time_frames[-1])} us")
            diag_n_frame = n_frame
            diag_t = time.time()


class HandStateMachine(StateMachine):
    unconnected = State(MoCapHandStateEnum.UNCONNECTED, initial=True)
    connected = State(MoCapHandStateEnum.CONNECTED)
    exited = State(MoCapHandStateEnum.EXITED, final=True)

    # transitions
    connect = (
            unconnected.to(exited, cond="hand_shutdown_requested")
            | unconnected.to(connected, cond="grpc_client_connected")
            | unconnected.to(unconnected, unless=["grpc_client_connected"])
    )

    spin = (
            connected.to(exited, cond="hand_shutdown_requested")
            | connected.to(connected, cond="grpc_client_connected")
            | connected.to(unconnected, unless=["grpc_client_connected"])
    )

    loop = (
            connect
            | spin
    )

    def grpc_client_connected(self):
        if self.current_state.value == self.connected.value:
            return self._thread is not None and self._thread.is_alive()
        else:
            return self._stub is not None

    def hand_shutdown_requested(self):
        return self._flags.shutdown

    def __init__(self, config: MoCapHandConfig | None = None, mano_state: ManoState = ManoState()):
        self._stub: imu_packet_pb2_grpc.IMUPacketServiceStub | None = None
        self._buffer = RingBuffer(size=1024, fps=config.fps)
        self._thread: threading.Thread | None = None
        self._kill_switch: threading.Event = threading.Event()

        # store config and mano_state
        self._config = config
        self._mano_state = mano_state

        # read saved params
        if self._config.param_inst_calib is not None:
            self._mano_state.raw2mano_calibrator = Raw2MANOCalibrator.decode(self._config.param_inst_calib)
        if self._config.param_shape_calib is not None:
            self._mano_state.shape_params_cpt[:] = torch.tensor(self._config.param_shape_calib)
        if self._config.param_wrist_calib:
            self._mano_state.wrist_calibrator = WristPoseCalibrator.decode(self._config.param_wrist_calib)

        # store flags
        self._flags = HandStateMachineFlags()

        super().__init__(allow_event_without_transition=True)

    @property
    def buffer(self) -> RingBuffer:
        return self._buffer

    @property
    def fps(self) -> int:
        return self._config.fps

    @property
    def mano_state(self):
        return self._mano_state

    @property
    def hand_id(self):
        return self._mano_state.hand_id

    def shutdown(self):
        self._flags.shutdown = True

    def update_root_calibrator(self, trigger_type: MoCapCalibrateRootType):
        """
        This function is used to update the root calibrator, the first calibrator

        :param trigger_type: MoCapCalibrateRootType
        """
        logger.debug(f"{trigger_type} root calibration triggered")

        if trigger_type == MoCapCalibrateRootType.RESET:
            # reset all calibration parameters
            self._mano_state.calib_root_horizontal_pose = trivial_mano_pose
            self._mano_state.calib_root_vertical_up_pose = trivial_mano_pose
            self._mano_state.calib_root_vertical_down_pose = trivial_mano_pose
            self._mano_state.raw2mano_calibrator = None
            return
        elif trigger_type == MoCapCalibrateRootType.CALC:
            # calculate the calibration parameters
            try:
                # apply calibration
                self._mano_state.raw2mano_calibrator = Raw2MANOCalibrator(
                    CalibrationQuatSet(
                        base_pose=self._mano_state.calib_root_horizontal_pose.as_quat(),
                        z_rot_pose_1=self._mano_state.calib_root_vertical_up_pose.as_quat(),
                        z_rot_pose_2=self._mano_state.calib_root_vertical_down_pose.as_quat()
                    ),
                    side=self._mano_state.hand_id
                )
                # store to config
                self._config.param_inst_calib = self._mano_state.raw2mano_calibrator.encode()
            except Exception as e:
                logger.exception(f"root calibration calculation error: {e}")
            return

        # get the averaged hand pose input, save to the corresponding pose
        calibrate_pose = get_averaged_hand_pose_input(self._buffer, 10)
        if trigger_type == MoCapCalibrateRootType.HORIZONTAL:
            self._mano_state.calib_root_horizontal_pose = calibrate_pose
        elif trigger_type == MoCapCalibrateRootType.VERTICAL_UP:
            self._mano_state.calib_root_vertical_up_pose = calibrate_pose
        elif trigger_type == MoCapCalibrateRootType.VERTICAL_DOWN:
            self._mano_state.calib_root_vertical_down_pose = calibrate_pose
        else:
            logger.error(f"unknown root calibration type: {trigger_type}")

    def update_installation_calibrator(self, trigger_type: MoCapCalibrateInstallationType):
        """
        This function is used to update the installation calibrator, the second calibrator (optional)

        :param trigger_type: MoCapCalibrateInstallationType

        WARNING: this function is not implemented yet
        """
        logger.debug(f"{trigger_type} installation calibration triggered")
        if not self.mano_state.raw2mano_calibrator:
            logger.error("please calibrate root first")
            return

        if trigger_type == MoCapCalibrateInstallationType.RESET:
            # reset all calibration parameters
            self._mano_state.calib_inst_pose_0 = trivial_mano_pose
            self._mano_state.calib_inst_pose_1 = trivial_mano_pose
            self._mano_state.calib_inst_pose_2 = trivial_mano_pose
            self._mano_state.calib_inst_mano_pose_0 = trivial_mano_tensor
            self._mano_state.calib_inst_mano_pose_1 = trivial_mano_tensor
            self._mano_state.calib_inst_mano_pose_2 = trivial_mano_tensor

            return
        elif trigger_type == MoCapCalibrateInstallationType.CALC:
            # calculate the calibration parameters
            # TODO: replace with amp and bias model
            return

        # get the averaged hand pose input, save to the corresponding pose
        calibrate_pose = get_averaged_hand_pose_input(self._buffer)
        if trigger_type == MoCapCalibrateInstallationType.POSE_0:
            self._mano_state.calib_inst_pose_0 = calibrate_pose
            self._mano_state.calib_inst_mano_pose_0 = self.mano_state.raw2mano_calibrator.get_mano_param(calibrate_pose)
        elif trigger_type == MoCapCalibrateInstallationType.POSE_1:
            self._mano_state.calib_inst_pose_1 = calibrate_pose
            self._mano_state.calib_inst_mano_pose_1 = self.mano_state.raw2mano_calibrator.get_mano_param(calibrate_pose)
        elif trigger_type == MoCapCalibrateInstallationType.POSE_2:
            self._mano_state.calib_inst_pose_2 = calibrate_pose
            self._mano_state.calib_inst_mano_pose_2 = self.mano_state.raw2mano_calibrator.get_mano_param(calibrate_pose)
        else:
            logger.error(f"unknown installation calibration type: {trigger_type}")

    def update_shape_calibrator(self, trigger_type: MoCapCalibrateShapeType):
        """
        This function is used to update the shape calibrator, the third calibrator.
        It optimizes mano shape parameter using the collected pose

        :param trigger_type: MoCapCalibrateShapeType
        """
        logger.debug(f"{trigger_type} shape calibration triggered")
        all_gathered_pose = [
            self._mano_state.calib_shape_index,
            self._mano_state.calib_shape_middle,
            self._mano_state.calib_shape_ring,
            self._mano_state.calib_shape_little
        ]
        if not self.mano_state.raw2mano_calibrator:
            logger.error("please calibrate root first")
            return

        if trigger_type == MoCapCalibrateShapeType.RESET:
            # reset all calibration parameters
            [x.clear() for x in all_gathered_pose]
            return
        elif trigger_type == MoCapCalibrateShapeType.CALC:
            if any([not x for x in all_gathered_pose]):
                logger.error("please calibrate all finger shapes first")
                return

            # assemble training set
            joints_rot_batch = torch.cat([
                torch.stack([
                    self._mano_state.raw2mano_calibrator.get_mano_param(_x) for _x in x
                ]) for x in all_gathered_pose
            ])
            finger_category = torch.cat(
                [torch.ones(len(x), dtype=torch.long) * i + 2 for i, x in enumerate(all_gathered_pose)]
            )
            # apply calibration
            self._mano_state.shape_params_cpt[:] = get_optimized_shape_params(
                joints_rot_batch,
                finger_category,
                self._mano_state.hand_id.value,  # left and right algorithms are different
                compute_device=self._mano_state.compute_device
            )
            # store to config
            self._config.param_shape_calib = self._mano_state.shape_params_cpt.detach().cpu().numpy().tolist()
            return

        # get the averaged hand pose input, save to the corresponding pose
        calibrate_pose_seq = get_seq_hand_pose_input(self._buffer)
        if trigger_type == MoCapCalibrateShapeType.INDEX:
            self._mano_state.calib_shape_index.extend(calibrate_pose_seq)
        elif trigger_type == MoCapCalibrateShapeType.MIDDLE:
            self._mano_state.calib_shape_middle.extend(calibrate_pose_seq)
        elif trigger_type == MoCapCalibrateShapeType.RING:
            self._mano_state.calib_shape_ring.extend(calibrate_pose_seq)
        elif trigger_type == MoCapCalibrateShapeType.LITTLE:
            self._mano_state.calib_shape_little.extend(calibrate_pose_seq)
        else:
            logger.error(f"unknown shape calibration type: {trigger_type}")

    def update_shape_keypoint_calibrator(self, trigger_type: MoCapCalibrateShapeKeyPointType):
        """
        This function is used to update the shape calibrator with keypoint, the third calibrator.
        It optimizes mano shape parameter using the collected pose

        :param trigger_type: MoCapCalibrateShapeKeyPointType
        """
        from modules.mocap import MoCap  # avoid circular import

        logger.warning(f"{trigger_type} shape calibration triggered, handle this !!!")
        if not self.mano_state.raw2mano_calibrator:
            logger.error("please calibrate root first")
            return

        if trigger_type == MoCapCalibrateShapeKeyPointType.RESET:
            # reset all calibration parameters
            self._mano_state.calib_shape_keypoints = np.zeros(shape=(len(MoCapCalibrateShapeKeyPointType), 3))
            return
        elif trigger_type == MoCapCalibrateShapeKeyPointType.CALC:
            input_points = torch.from_numpy(self._mano_state.calib_shape_keypoints[1:22]).to(torch.float32)
            # with out the T_MCP
            output = self._mano_state.mano_layer(torch.zeros(1, 48).cuda(), torch.randn(1, 10).cuda()).verts.squeeze(0)
            output = output[keypoints_verts_ids]
            input_points_aligned = point_match(input_points, output)
            get_optimized_shape_params_from_keypoints(input_points_aligned, keypoints_verts_ids, 'right', iter_num=500)
        else:
            pos = get_averaged_tracker_position_input(MoCap().wrist_tracker.buffer, MoCapHandIDEnum.SPIKE)
            keypoint_name, keypoint_index = trigger_type.value
            self._mano_state.calib_shape_keypoints[keypoint_index] = pos

    def update_wrist_calibrator(self, trigger_type: MoCapCalibrateWristType):
        """
        This function is used to update the wrist calibrator, the fourth calibrator.
        It is used when external tracker is ready.
        """
        from modules.mocap import MoCap  # avoid circular import

        logger.debug(f"{trigger_type} wrist calibration triggered")
        if trigger_type == MoCapCalibrateWristType.RESET:
            # reset all calibration parameters
            self._mano_state.calib_wrist_thumb_tip_pos = trivial_position
            self._mano_state.calib_wrist_index_tip_pos = trivial_position
            self._mano_state.calib_wrist_middle_tip_pos = trivial_position
            self._mano_state.calib_wrist_ring_tip_pos = trivial_position
            self._mano_state.calib_wrist_little_tip_pos = trivial_position
            self._mano_state.wrist_calibrator = None
            return
        elif trigger_type == MoCapCalibrateWristType.CALC:
            # calculate the calibration parameters

            # warning: do not move hand during calibration
            # assemble calibration information
            hand_tracker_pos = get_averaged_tracker_position_input(MoCap().wrist_tracker.buffer,
                                                                   self._mano_state.hand_id)
            hand_tracker_rot = get_averaged_tracker_rotation_input(MoCap().wrist_tracker.buffer,
                                                                   self._mano_state.hand_id)
            distal_pos_mano = self._mano_state.get_distal_pos()
            distal_pos_tracker = np.stack([
                self._mano_state.calib_wrist_thumb_tip_pos,
                self._mano_state.calib_wrist_index_tip_pos,
                self._mano_state.calib_wrist_middle_tip_pos,
                self._mano_state.calib_wrist_ring_tip_pos,
                self._mano_state.calib_wrist_little_tip_pos
            ])
            # apply calibration
            self._mano_state.wrist_calibrator = WristPoseCalibrator(
                hand_tracker_rot,
                hand_tracker_pos,
                distal_pos_mano,
                distal_pos_tracker
            )
            # store to config
            self._config.param_wrist_calib = self._mano_state.wrist_calibrator.encode()
            return

        # get the averaged hand pose input, save to the corresponding pose
        pos = get_averaged_tracker_position_input(MoCap().wrist_tracker.buffer, MoCapHandIDEnum.SPIKE)
        if pos is None:
            logger.error("no wrist position data")
            return
        if trigger_type == MoCapCalibrateWristType.THUMB:
            self._mano_state.calib_wrist_thumb_tip_pos = pos
        elif trigger_type == MoCapCalibrateWristType.INDEX:
            self._mano_state.calib_wrist_index_tip_pos = pos
        elif trigger_type == MoCapCalibrateWristType.MIDDLE:
            self._mano_state.calib_wrist_middle_tip_pos = pos
        elif trigger_type == MoCapCalibrateWristType.RING:
            self._mano_state.calib_wrist_ring_tip_pos = pos
        elif trigger_type == MoCapCalibrateWristType.LITTLE:
            self._mano_state.calib_wrist_little_tip_pos = pos

    def on_enter_unconnected(self):
        # if the hand is not enabled
        if not self._config.enabled:
            self._flags.shutdown = True
            return

        # if the thread was created
        if self._thread is not None:
            if self._thread.is_alive():
                self._kill_switch.set()
                self._thread.join()
            else:
                self._thread = None

        # Establish connection to IMU gRPC server
        try:
            channel = grpc.insecure_channel(self._config.grpc_address, options=[("grpc.default_deadline", 10)], )
            self._stub = imu_packet_pb2_grpc.IMUPacketServiceStub(channel)
        except Exception as e:
            logger.exception(e)
            time.sleep(1)
            self._stub = None
            return

        # WakeUp the server and test its function
        try:
            response = self._stub.GetFIFOStatus(imu_packet_pb2.Empty(), timeout=0.1)
            if not response.status:
                self._stub.SetFIFOStatus(imu_packet_pb2.IMUSetFIFOStatusRequest(status=True))
                return
        except Exception as e:
            logger.warning(f'rpc error, device not connected {e}')
            self._stub = None
        finally:
            time.sleep(0.1)

    def on_enter_connected(self):
        # start the background thread
        if self._thread is None:
            t = threading.Thread(
                target=hand_grpc_receiver,
                args=(
                    self._kill_switch,
                    self._stub,
                    self._buffer
                )
            )
            t.start()
            self._thread = t

    def on_enter_exited(self):
        # clean up
        self._kill_switch.set()
        if self._thread is not None:
            if self._thread.is_alive():
                self._thread.join(timeout=1)  # timeout 1s to wait the thread to stop, then skip

        self._stub = None
        self._buffer.reset()
        self._thread = None
        self._kill_switch.clear()
