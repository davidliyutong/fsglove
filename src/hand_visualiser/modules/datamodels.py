import json
import os
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R
from loguru import logger
from pydantic import BaseModel, Field, model_validator

from modules.enumerates import MoCapCalibrateInstallationType, MoCapCalibrateShapeType, MoCapHandIDEnum, \
    MoCapCalibrateRootType, MoCapCalibrateWristType, MoCapCalibrateShapeKeyPointType


class MoCapHandConfig(BaseModel):
    grpc_address: str | None = None
    fps: int | None = None
    # parameters related to calibration
    param_rot_bias: list[list[float]] | None = None
    param_rot_amp: list[float] | None = None
    param_inst_calib: dict[str, Any] | None = None
    param_shape_calib: list[float] | None = None
    param_wrist_calib: dict[str, Any] | None = None

    @model_validator(mode='after')
    def verify(self):
        if self.grpc_address:
            if not self.fps:
                raise ValueError("fps is required if grpc_address is provided")
        return self

    @property
    def enabled(self) -> bool:
        return bool(self.grpc_address)


class MoCapNokovConfig(BaseModel):
    server_ip: str | None = None
    fps: int | None = None
    # TODO: add an stl mapping
    rigidbody_id_mapping: dict[MoCapHandIDEnum, str | int] | None = Field(
        default_factory=lambda: {MoCapHandIDEnum.LEFT: 'left', MoCapHandIDEnum.RIGHT: 'right', MoCapHandIDEnum.SPIKE: 'spike'})
    record_markers: bool = False

    @model_validator(mode='after')
    def verify(self):
        if self.server_ip:
            if not self.fps:
                raise ValueError("fps is required if server_ip is provided")
        assert MoCapHandIDEnum.SPIKE in self.rigidbody_id_mapping.keys(), "SPIKE rigid body id is required"
        return self

    @property
    def enabled(self) -> bool:
        return bool(self.server_ip)

class MoCapNokovGeneralConfig(BaseModel):
    server_ip: str | None = None
    fps: int | None = None
    rigidbody_id_mapping: dict[str, str] | None = Field(default_factory=lambda: {})
    record_markers: bool = False

    @model_validator(mode='after')
    def verify(self):
        if self.server_ip:
            if not self.fps:
                raise ValueError("fps is required if server_ip is provided")
        return self

    @property
    def enabled(self) -> bool:
        return bool(self.server_ip)

class MoCapShadowConfig(BaseModel):
    shadow_offset: list[float] | None = Field(default_factory=lambda : [0.0] * 24)
    shadow_amplitude: list[float] | None = Field(default_factory=lambda : [1.0] * 24)
    endpoint: str | None = None


class MoCapConfig(BaseModel):
    # general configs
    _loaded: bool = False
    use_rfu: bool = False
    use_cuda: bool = True
    debug: bool = False
    show_tracker: bool = False
    enable_control: bool = False
    enable_api: bool = False
    api_port: int = 8000

    # sub configs
    wrist_tracker_cfg: MoCapNokovConfig | MoCapNokovGeneralConfig | None = None
    left_hand_config: MoCapHandConfig | None = None
    right_hand_config: MoCapHandConfig | None = None
    shadow_config: MoCapShadowConfig = MoCapShadowConfig()

    # datastore
    data_store: str = './'

    # source
    local_path: str = './config.json'

    @classmethod
    def read_from_disk(cls, local_path: str) -> tuple[Any, Exception | None]:
        if os.path.exists(local_path):
            try:
                with open(local_path, 'r') as f:
                    config = json.load(f)
                res = MoCapConfig(**config)
                res.local_path = local_path
                logger.info(f"config file loaded from {local_path}")
                return res, None
            except Exception as e:
                return None, e
        else:
            logger.warning("config file not found, creating new one")
            config = cls(
                wrist_tracker_cfg=MoCapNokovGeneralConfig(),
                left_hand_config=MoCapHandConfig(),
                right_hand_config=MoCapHandConfig(),
                shadow_config=MoCapShadowConfig(),
                local_path=local_path,
            )
            with open(local_path, 'w') as f:
                json.dump(config.model_dump(), f, indent=4)
            return config, None

    def save_to_disk(self, path: str = None) -> Exception | None:
        if path is None:
            path = self.local_path
        with open(path, 'w') as f:
            json.dump(self.model_dump(), f, indent=4)
        logger.info(f"config dumped to {path}")
        return None

class MoCapGUIEvents(BaseModel):
    # general control flags
    mocap_exit_requested: bool = False
    mocap_save_config_triggered: bool = False

    # flags related to installation calibration
    mocap_calibrate_root_triggered: bool = False
    mocap_calibrate_root_type: MoCapCalibrateRootType = MoCapCalibrateRootType.HORIZONTAL
    mocap_calibrate_root_target: MoCapHandIDEnum = MoCapHandIDEnum.UNKNOWN

    # flags related to installation calibration
    mocap_calibrate_installation_triggered: bool = False
    mocap_calibrate_installation_type: MoCapCalibrateInstallationType = MoCapCalibrateInstallationType.POSE_0
    mocap_calibrate_installation_target: MoCapHandIDEnum = MoCapHandIDEnum.UNKNOWN

    # flags related to shape calibration
    mocap_calibrate_shape_triggered: bool = False
    mocap_calibrate_shape_type: MoCapCalibrateShapeType = MoCapCalibrateShapeType.INDEX
    mocap_calibrate_shape_target: MoCapHandIDEnum = MoCapHandIDEnum.UNKNOWN
    mocap_calibrate_shape_keypoint_triggered: bool = False
    mocap_calibrate_shape_keypoint_type: MoCapCalibrateShapeKeyPointType = None
    mocap_calibrate_shape_keypoint_target: MoCapHandIDEnum = MoCapHandIDEnum.UNKNOWN

    # flags related to wrist calibration
    mocap_calibrate_wrist_triggered: bool = False
    mocap_calibrate_wrist_type: MoCapCalibrateWristType = MoCapCalibrateWristType.THUMB
    mocap_calibrate_wrist_target: MoCapHandIDEnum = MoCapHandIDEnum.UNKNOWN

    # flags related to rfu.ctrl
    mocap_rfu_ctrl_launch_triggered: bool = False
    mocap_rfu_ctrl_launch_target: MoCapHandIDEnum = MoCapHandIDEnum.UNKNOWN
    mocap_rfu_ctrl_termination_triggered: bool = False

    # flags related to rfu.retarget
    mocap_rfu_retarget_open_triggered: bool = False
    mocap_rfu_retarget_open_target: MoCapHandIDEnum = MoCapHandIDEnum.UNKNOWN
    mocap_rfu_retarget_next_triggered: bool = False
    mocap_rfu_retarget_save_triggered: bool = False
    mocap_rfu_retarget_close_triggered: bool = False

    # flags related to recording
    mocap_start_recording_triggered: bool = False
    mocap_stop_recording_triggered: bool = False

    def __hash__(self):
        return hash(tuple(self.model_dump().values()))

class ImuMsg:
    sys_ticks: int
    imu_rotation: R
    seq: int

    def __init__(self, sys_ticks: int, imu_rotation: R, seq=0):
        """
        :param: sys_ticks: int
        :param: imu_quat: Rotation, with shape (16,)
        """
        self.sys_ticks = sys_ticks
        self.imu_rotation = imu_rotation
        self.seq = seq

    def to_dict(self):
        return {
            'sys_ticks': self.sys_ticks,
            'imu_rotation': self.imu_rotation.as_quat().tolist(),
            'seq': self.seq,
        }

    def __repr__(self):
        return f"<ImuMsg: {self.seq}>"

class  WristTrackerMsg:
    timestamp_us: int
    pos: np.ndarray
    rot: R
    markers: np.ndarray
    seq: int

    def __init__(self, timestamp_us, pos, rot, markers, seq=0):
        """
        :param: timestamp_us: int
        :param: pos: np.ndarray, with shape (3,)
        :param: rot: Rotation, with shape (1,)
        """
        self.timestamp_us = timestamp_us
        self.pos = pos
        self.rot = rot
        self.markers = markers
        self.seq = seq

    def to_dict(self):
        return {
            'timestamp_us': self.timestamp_us,
            'pos': self.pos.tolist(),
            'rot': self.rot.as_quat().tolist(),
            'markers': self.markers.tolist() if self.markers is not None else None,
            'seq': self.seq,
        }

    def __repr__(self):
        return f"<WristTrackerMsg pos={self.pos}, rot={self.rot.as_quat().tolist()}, seq={self.seq}, markers={self.markers}>"


class PackedWristTrackerMsg:
    timestamp_us: int
    tracked_objects: dict[str, WristTrackerMsg] | None
    seq: int

    def __init__(self, timestamp_us, seq=0, **tracked_objects):
        self.timestamp_us = timestamp_us
        self.tracked_objects = tracked_objects
        self.seq = seq

    def to_dict(self):
        return {
            'timestamp_us': self.timestamp_us,
            'tracked_objects': {k: v.to_dict() if v is not None else None for k, v in self.tracked_objects.items()},
            'seq': self.seq,
        }
