import time
import threading
import numpy as np

from loguru import logger
from pydantic import BaseModel
from nokov.nokovsdk import PySDKClient, POINTER, DataDescriptions, DataDescriptors
from statemachine import StateMachine, State
from scipy.spatial.transform import Rotation as R

from modules.ring_buffer import RingBuffer
from modules.enumerates import MoCapTrackerStateEnum
from modules.datamodels import WristTrackerMsg, PackedWristTrackerMsg, MoCapNokovConfig, MoCapNokovGeneralConfig
from modules.utils import CoordinateFrame


class NokovStateMachineFlags(BaseModel):
    compute_mano: bool = False
    shutdown: bool = False
    server_connected: bool = False


def nokov_receiver(
        kill_switch: threading.Event,
        client: PySDKClient,
        rigid_body_mapping: dict[str, str],
        record_markers: bool,
        buffer: RingBuffer):
    start_t = time.time()
    n_frame = 0

    diag_t = start_t
    diag_n_frame = n_frame

    pre_frame_idx = 0

    # get data description
    pdds = POINTER(DataDescriptions)()
    client.PyGetDataDescriptions(pdds)
    dataDefs = pdds.contents
    rigidbody_id_to_name_mapping: dict[int, str] = {}  # maps nokov ID(int) to rigidbody name
    skeleton_id_to_name_mapping: dict[int, str] = {}  # maps nokov ID(int) to skeleton name
    rigid_body_mapping_inv: dict[str, str] = {v:k for k, v in rigid_body_mapping.items()}

    for i_def in range(dataDefs.nDataDescriptions):
        dataDef = dataDefs.arrDataDescriptions[i_def]
        if dataDef.type == DataDescriptors.Descriptor_RigidBody.value:
            rigidBody = dataDef.Data.RigidBodyDescription.contents
            rigidbody_id_to_name_mapping[rigidBody.ID] = rigidBody.szName.decode('utf-8')
        if dataDef.type == DataDescriptors.Descriptor_Skeleton.value:
            skeleton = dataDef.Data.SkeletonDescription.contents
            skeleton_id_to_name_mapping[skeleton.skeletonID] = skeleton.szName.decode('utf-8')

    while True:
        if kill_switch.is_set():
            break
        frame = client.PyGetLastFrameOfMocapData()
        if not frame: continue

        # for debug skeleton purpose
        # skeleton = frame.contents.Skeletons[0]
        # data = skeleton.RigidBodyData[1]
        # print([data.x, data.y, data.z])

        try:
            if frame is None: continue
            frame_data = frame.contents
            if frame_data.iFrame == pre_frame_idx: continue
            pre_frame_idx = frame_data.iFrame
            n_frame += 1

            # process timestamp
            length = 128
            szTimeCode = bytes(length)
            client.PyTimecodeStringify(frame_data.Timecode, frame_data.TimecodeSubframe, szTimeCode, length)
            time_stamp = frame_data.iTimeStamp

            rigidbody_data: dict[str, WristTrackerMsg | None] = {}

            # process rigid body data
            for idx in range(frame_data.nRigidBodies):
                # process rigidbody
                rigidbody = frame_data.RigidBodies[idx]
                name = rigidbody_id_to_name_mapping.get(rigidbody.ID)
                if name not in rigid_body_mapping_inv.keys(): continue
                position = np.asarray([rigidbody.x, rigidbody.y, rigidbody.z]) / 1000  # mm to m
                rotation = R.from_quat([rigidbody.qx, rigidbody.qy, rigidbody.qz, rigidbody.qw])
                markers = np.stack([
                    np.array(rigidbody.Markers[i]) for i in range(rigidbody.nMarkers)
                ]) if record_markers else None
                rigidbody_data[rigid_body_mapping_inv[name]] = WristTrackerMsg(
                    timestamp_us=time_stamp,
                    pos=position,
                    rot=rotation,
                    markers=markers,
                    seq=n_frame
                )

            # process skeleton data
            for idx in range(frame_data.nSkeletons):
                # process rigidbody
                skeleton = frame.contents.Skeletons[idx]
                for part_idx in range(skeleton.nRigidBodies):
                    name = f"{skeleton_id_to_name_mapping.get(skeleton.skeletonID)}.{str(part_idx)}"
                    if name not in rigid_body_mapping_inv.keys(): continue
                    skeleton_part = skeleton.RigidBodyData[part_idx]
                    position = np.asarray([skeleton_part.x, skeleton_part.y, skeleton_part.z]) / 1000  # mm to m
                    rotation = R.from_quat([skeleton_part.qx, skeleton_part.qy, skeleton_part.qz, skeleton_part.qw])
                    rigidbody_data[name] = WristTrackerMsg(
                        timestamp_us=time_stamp,
                        pos=position,
                        rot=rotation,
                        seq=n_frame
                    )

            buffer.push(
                PackedWristTrackerMsg(
                    timestamp_us=time_stamp,
                    seq=n_frame,
                    **rigidbody_data,
                )
            )

            # diagnostic
            if (time.time() - diag_t) >= 2:
                logger.info(
                    f"nokov refresh fps: {(n_frame - diag_n_frame) / (time.time() - diag_t)}")
                diag_n_frame = n_frame
                diag_t = time.time()
        finally:
            # release frame
            client.PyNokovFreeFrame(frame)


class NokovStateMachine(StateMachine):
    unconnected = State(MoCapTrackerStateEnum.UNCONNECTED, initial=True)
    connected = State(MoCapTrackerStateEnum.CONNECTED)
    exited = State(MoCapTrackerStateEnum.EXITED, final=True)

    # transitions
    connect = (
            unconnected.to(exited, cond="nokov_shutdown_requested")
            | unconnected.to(connected, cond="server_connected")
            | unconnected.to(unconnected, unless=["server_connected"])
    )

    spin = (
            connected.to(exited, cond="nokov_shutdown_requested")
            | connected.to(connected, cond="server_connected")
            | connected.to(unconnected, unless=["server_connected"])
    )

    loop = (
            connect
            | spin
    )

    def server_connected(self):
        if self.current_state.value == self.connected.value:
            return self._thread is not None and self._thread.is_alive()
        else:
            return self._flags.server_connected

    def nokov_shutdown_requested(self):
        return self._flags.shutdown

    def __init__(self, config: MoCapNokovConfig | MoCapNokovGeneralConfig):
        # init state machine
        self._config = config
        self._server_ip = config.server_ip
        self._client = PySDKClient()
        self._buffer = RingBuffer(size=1024, fps=config.fps)
        self._thread: threading.Thread | None = None
        self._kill_switch: threading.Event = threading.Event()
        self._rigidbody_id_mapping = config.rigidbody_id_mapping

        # coordinate frame for visualization, accessed by GUI
        self._spike_coordinate_frame_vis = CoordinateFrame(key='spike')

        # store flags
        self._flags = NokovStateMachineFlags()

        super().__init__(allow_event_without_transition=True)

    @property
    def buffer(self):
        return self._buffer

    @property
    def fps(self) -> int:
        return self._config.fps

    @property
    def spike_coordinate_frame_vis(self) -> CoordinateFrame:
        return self._spike_coordinate_frame_vis

    def shutdown(self):
        self._flags.shutdown = True

    def on_enter_unconnected(self):
        # if the thread was created
        if self._thread is not None:
            if self._thread.is_alive():
                self._kill_switch.set()
                self._thread.join()
            else:
                self._thread = None

        ret = self._client.Initialize(bytes(self._server_ip, encoding="utf8"))
        if ret == 0:
            self._flags.server_connected = True

    def on_enter_connected(self):
        # start the background thread
        if self._thread is None:
            t = threading.Thread(
                target=nokov_receiver,
                args=(
                    self._kill_switch,
                    self._client,
                    self._rigidbody_id_mapping,
                    self._config.record_markers,
                    self._buffer,
                )
            )
            t.start()
            self._thread = t

    def on_enter_exited(self):
        # clean up
        self._kill_switch.set()
        if self._thread is not None:
            if self._thread.is_alive():
                self._thread.join()
        self._buffer.reset()
        self._thread = None
        self._kill_switch.clear()
