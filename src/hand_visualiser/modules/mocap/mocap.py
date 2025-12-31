import json
import os
import threading
import time
from uuid import uuid4

import numpy as np
import torch
from chumpy import shape
from loguru import logger
from statemachine import StateMachine, State

from modules.dataflow import input_synchronizer
from modules.datamodels import MoCapConfig
from modules.enumerates import MoCapStateEnum, MoCapThreadPluginEnum, MoCapHandStateEnum, MoCapHandIDEnum
from modules.hand import HandStateMachine
from modules.mano_model import ManoState
from modules.mocap.thread_manager import MoCapThreadManager
from modules.teleop.rfu_ctrl import rfu_controller
from modules.teleop.rfu_retarget import rfu_retarget_trainer
from modules.teleop.state import ShadowState
from modules.wrist_tracker.nokov import NokovStateMachine as WristTrackerStateMachine
from modules.recording import jsonl_writer
from modules.ring_buffer import RingBuffer
from modules.utils import singleton, progress_statemachine


@singleton
class MoCap(StateMachine):
    """
    MoCapStateMachine
    """
    initialized = State(MoCapStateEnum.INITIALIZED, initial=True)
    unconnected = State(MoCapStateEnum.UNCONNECTED)
    connected = State(MoCapStateEnum.CONNECTED)
    running = State(MoCapStateEnum.RUNNING)
    exited = State(MoCapStateEnum.EXITED)
    finalized = State(MoCapStateEnum.FINALIZED, final=True)

    # transitions
    init_gui = (
        initialized.to(unconnected, cond="mocap_is_running")
    )

    connect_device = (
            unconnected.to(connected, cond="clients_are_spinning")
            | unconnected.to(exited, unless="mocap_is_running")
            | unconnected.to(unconnected, unless="clients_are_spinning")

    )

    spin = (
            connected.to(running, cond="clients_are_spinning")
            | running.to(unconnected, unless="clients_are_spinning")
            | running.to(exited, unless="mocap_is_running")
            | running.to(running, cond="clients_are_spinning")
    )

    finalize = exited.to(finalized)

    loop = (
            init_gui
            | connect_device
            | spin
            | finalize
    )

    def mocap_is_running(self):
        return not self.shutdown_requested

    def clients_are_spinning(self):
        return all([
            (self._left_hand and self._left_hand.current_state.value == MoCapHandStateEnum.CONNECTED.value) or not self._left_hand,
            (self._right_hand and self._right_hand.current_state.value == MoCapHandStateEnum.CONNECTED.value) or not self._right_hand,
            (self._wrist_tracker and self._wrist_tracker.current_state.value == MoCapHandStateEnum.CONNECTED.value) or not self._wrist_tracker
        ])

    def __init__(self, config: MoCapConfig):
        logger.info("launching MoCap application")
        self._config = config
        self._thread_manager = MoCapThreadManager()

        # set compute device
        if self._config.use_cuda:
            device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        else:
            device = torch.device('cpu')
        self._compute_device = device
        logger.info(f'Using device: {device}')

        # init submodules
        self._left_hand = HandStateMachine(
            self._config.left_hand_config,
            ManoState(identifier=str(MoCapHandIDEnum.LEFT), hand_id=MoCapHandIDEnum.LEFT, device=self._compute_device)
        ) if self._config.left_hand_config else None
        self._right_hand = HandStateMachine(
            self._config.right_hand_config,
            ManoState(identifier=str(MoCapHandIDEnum.RIGHT), hand_id=MoCapHandIDEnum.RIGHT, device=self._compute_device)
        ) if self._config.right_hand_config else None
        self._wrist_tracker = WristTrackerStateMachine(self._config.wrist_tracker_cfg) if self._config.wrist_tracker_cfg else None
        self._shadow_state = ShadowState.from_shadow_config(self._config.shadow_config)
        self._core_buf = RingBuffer(2000, 100)

        # dataflow mapping
        self._ring_buffer_mapping = dict()
        self._fps = []  # collection of fps from all enabled streams
        if self._left_hand:
            self._ring_buffer_mapping[MoCapThreadPluginEnum.LEFT_HAND_RPC] = self._left_hand.buffer
            self._fps.append(self._left_hand.fps)
        if self._right_hand:
            self._ring_buffer_mapping[MoCapThreadPluginEnum.RIGHT_HAND_RPC] = self._right_hand.buffer
            self._fps.append(self._right_hand.fps)
        if self._wrist_tracker:
            self._ring_buffer_mapping[MoCapThreadPluginEnum.WRIST_TRACKER_SDK] = self._wrist_tracker.buffer
            self._fps.append(self._wrist_tracker.fps)

        # other flags
        self.shutdown_requested = False

        super().__init__(allow_event_without_transition=True)

    @property
    def thread_manager(self):
        return self._thread_manager

    @property
    def compute_device(self):
        return self._mano_state.compute_device

    @property
    def left_hand(self) -> HandStateMachine:
        return self._left_hand

    @property
    def right_hand(self) -> HandStateMachine:
        return self._right_hand

    @property
    def wrist_tracker(self) -> WristTrackerStateMachine:
        return self._wrist_tracker

    @property
    def shadow_state(self) -> ShadowState:
        return self._shadow_state

    @property
    def is_recording(self):
        return self._thread_manager.get_thread(MoCapThreadPluginEnum.RECORDING_LOCAL) is not None

    def get_hand_by_id(self, hand_id: MoCapHandIDEnum):
        if hand_id == MoCapHandIDEnum.LEFT:
            return self.left_hand
        if hand_id == MoCapHandIDEnum.RIGHT:
            return self.right_hand

    @property
    def config(self):
        return self._config

    @property
    def enabled_streams(self) -> list[MoCapThreadPluginEnum]:
        return list(self._ring_buffer_mapping.keys())

    @property
    def buffer(self):
        return self._core_buf

    def shutdown(self):
        # shutdown the system

        logger.info("shutting down hands and optitrack")
        if self._left_hand:
            self.left_hand.shutdown()
            progress_statemachine(self._left_hand, MoCapHandStateEnum.EXITED)

        if self._right_hand:
            self.right_hand.shutdown()
            progress_statemachine(self._right_hand, MoCapHandStateEnum.EXITED)

        if self._wrist_tracker:
            self.wrist_tracker.shutdown()
            progress_statemachine(self._wrist_tracker, MoCapHandStateEnum.EXITED)
        self.shutdown_requested = True

    def on_enter_initialized(self):
        logger.debug(f"on_enter_initialized")

        # start synchronize thread
        self._thread_manager.reset(MoCapThreadPluginEnum.SYNC_LOCAL)
        t = threading.Thread(
            target=input_synchronizer,
            args=(
                self._thread_manager.get_switch(MoCapThreadPluginEnum.SYNC_LOCAL),
                self._ring_buffer_mapping,
                self._core_buf
            )
        )
        self._thread_manager.attach(MoCapThreadPluginEnum.SYNC_LOCAL, t, start=True)

    def on_enter_unconnected(self):
        logger.debug(f"on_enter_unconnected")
        self.spin_clients()
        time.sleep(1)

    def spin_clients(self):
        [m.loop() for m in [self._left_hand, self._right_hand, self._wrist_tracker] if m]

    def start_recording(self):
        self._thread_manager.reset(MoCapThreadPluginEnum.RECORDING_LOCAL)
        t = threading.Thread(
            target=jsonl_writer,
            args=(
                self._thread_manager.get_switch(MoCapThreadPluginEnum.RECORDING_LOCAL),
                self._ring_buffer_mapping,
            )
        )
        self._thread_manager.attach(MoCapThreadPluginEnum.RECORDING_LOCAL, t, start=True)
        time.sleep(0.1)

    def stop_recording(self):
        self._thread_manager.reset(MoCapThreadPluginEnum.RECORDING_LOCAL)
        time.sleep(0.1)

    def launch_rfu_control(self, hand_id: MoCapHandIDEnum):
        self._thread_manager.reset(MoCapThreadPluginEnum.RFU_CTRL_LOCAL)
        t = threading.Thread(
            target=rfu_controller,
            args=(
                self._thread_manager.get_switch(MoCapThreadPluginEnum.RFU_CTRL_LOCAL),
                hand_id,
                self._ring_buffer_mapping,
            )
        )
        self._thread_manager.attach(MoCapThreadPluginEnum.RFU_CTRL_LOCAL, t, start=True)
        time.sleep(0.1)

    def terminate_rfu_control(self):
        self._thread_manager.reset(MoCapThreadPluginEnum.RFU_CTRL_LOCAL)
        time.sleep(0.1)

    def open_rfu_retarget(self, hand_id: MoCapHandIDEnum):
        self._thread_manager.reset(MoCapThreadPluginEnum.RFU_RETARGET_LOCAL)
        t = threading.Thread(
            target=rfu_retarget_trainer,
            args=(
                self._thread_manager.get_switch(MoCapThreadPluginEnum.RFU_RETARGET_LOCAL),
                hand_id,
            )
        )
        self._thread_manager.attach(MoCapThreadPluginEnum.RFU_RETARGET_LOCAL, t, start=True)

    def save_rfu_retarget(self, hand_id: MoCapHandIDEnum):
        uid = uuid4()
        hand = self.get_hand_by_id(hand_id)

        with open(os.path.join(self._config.data_store, f"rfu_retarget_{hand_id}_{uid}.json"), 'w') as f:
            json.dump(
                {
                    "shadow": self.shadow_state.shadow_target_joint_positions.tolist(),
                    "mano": hand.mano_state.pose_params_cpt.detach().cpu().numpy().tolist(),
                }, f
            )

    def close_rfu_retarget(self):
        self._thread_manager.reset(MoCapThreadPluginEnum.RFU_RETARGET_LOCAL)
        time.sleep(0.1)

    def on_enter_connected(self):
        logger.debug(f"on_enter_connected, using: {self._compute_device}")

    def update_root_calibrator(self, hand_id: MoCapHandIDEnum, trigger_type):
        """
        Interface of updating root calibrator
        """
        if hand_id == MoCapHandIDEnum.LEFT:
            self._left_hand.update_root_calibrator(trigger_type)

        if hand_id == MoCapHandIDEnum.RIGHT:
            self._right_hand.update_root_calibrator(trigger_type)

    def update_installation_calibrator(self, hand_id: MoCapHandIDEnum, trigger_type):
        """
        Interface of updating installation calibrator
        """
        if hand_id == MoCapHandIDEnum.LEFT:
            self._left_hand.update_installation_calibrator(trigger_type)

        if hand_id == MoCapHandIDEnum.RIGHT:
            self._right_hand.update_installation_calibrator(trigger_type)

    def update_shape_calibrator(self, hand_id: MoCapHandIDEnum, trigger_type):
        """
        Interface of updating shape cal
        """
        if hand_id == MoCapHandIDEnum.LEFT:
            self._left_hand.update_shape_calibrator(trigger_type)

        if hand_id == MoCapHandIDEnum.RIGHT:
            self._right_hand.update_shape_calibrator(trigger_type)

    def update_shape_keypoint_calibrator(self, hand_id: MoCapHandIDEnum, trigger_type):
        """
        Interface of updating shape cal
        """
        if hand_id == MoCapHandIDEnum.LEFT:
            self._left_hand.update_shape_keypoint_calibrator(trigger_type)

        if hand_id == MoCapHandIDEnum.RIGHT:
            self._right_hand.update_shape_keypoint_calibrator(trigger_type)

    def update_wrist_calibrator(self, hand_id: MoCapHandIDEnum, trigger_type):
        """
        Interface of updating wrist calibrator
        """
        if hand_id == MoCapHandIDEnum.LEFT:
            self._left_hand.update_wrist_calibrator(trigger_type)

        if hand_id == MoCapHandIDEnum.RIGHT:
            self._right_hand.update_wrist_calibrator(trigger_type)

    def on_enter_running(self):
        """
        on_enter_running
        """
        logger.debug(f"on_enter_running")
        self.spin_clients()
        time.sleep(1)
