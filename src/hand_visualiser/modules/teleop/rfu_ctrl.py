import threading
import time

import numpy as np
import torch
import requests
from loguru import logger
from numpy import ndarray
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
from scipy.spatial.transform import Rotation as R
from pydantic import BaseModel

from modules.enumerates import MoCapThreadPluginEnum, MoCapHandIDEnum
from modules.ring_buffer import RingBuffer
from modules.utils import rot2joint_positions, mano_rotvec_to_rfu_euler

class Joints(BaseModel):
    positions: list[float] = [0.0] * 24

def rfu_controller(kill_switch: threading.Event, hand_id: MoCapHandIDEnum, in_buf_map: dict[str, RingBuffer]):
    from modules.mocap import MoCap # avoid circular import
    m = MoCap()
    stream_id = MoCapThreadPluginEnum.from_hand_id(hand_id)
    buffer = in_buf_map[stream_id]
    hand = m.get_hand_by_id(hand_id)
    if hand is None:
        logger.error(f"Hand {hand_id} not found")
        return

    # initialize variables
    try:
        env = RFUniverseBaseEnv()
    except Exception as e:
        logger.exception(e)
        return

    # intitialize client if configured
    if m.config.shadow_config.endpoint is not None:
        logger.info(f"connecting shadow controller at {m.config.shadow_config.endpoint}")
        session = requests.session()
    else:
        session = None

    # create o3d meshes/pcds
    shadow = env.LoadURDF(path='assets/gripper/shadow.urdf', native_ik=False)

    shadow.SetTransform(position=[0, 1, 0], rotation=[0, 90, 0])
    env.SetViewTransform(position=[0, 1.3, -0.4])
    env.step(1)
    env.ViewLookAt((np.asarray(shadow.data['position']) + np.asarray([0, 0.3, 0])).tolist())
    env.step(10)

    lower_limit = np.asarray(shadow.data['joint_lower_limit'])
    upper_limit = np.asarray(shadow.data['joint_upper_limit'])

    fps = 30
    n = 0

    diag_n = n
    diag_t = time.time()

    try:
        last_t = time.time()
        while True:
            if kill_switch.is_set():
                break

            hand_msg, _, err = buffer.peek()
            if hand_msg is None or hand.mano_state.raw2mano_calibrator is None:
                time.sleep(1/fps)
                continue

            # Hand pose
            mano_pose = hand.mano_state.raw2mano_calibrator.get_mano_param(hand_msg.imu_rotation, map_to_world=True)
            # mano_pose = torch.zeros(16,3)

            euler = mano_rotvec_to_rfu_euler(mano_pose) / np.pi * 180
            joint_positions = rot2joint_positions(euler)
            joint_positions = m.shadow_state.shadow_amplitude * joint_positions + m.shadow_state.shadow_offset
            clipped_positions: ndarray = np.clip(joint_positions, lower_limit, upper_limit)
            shadow.SetJointPosition(clipped_positions.tolist())

            if session is not None and m.config.enable_control:
                try:
                    session.post(
                        f"{m.config.shadow_config.endpoint}/joints/", 
                        json=Joints(positions=clipped_positions.tolist()).model_dump()
                    )
                except Exception as e:
                    logger.warning(e)

            # loop update
            env.step(1)
            n += 1

            # debug
            if (time.time() - diag_t) >= 10:
                logger.debug(f"rfu visualization fps: {(n - diag_n) / (time.time() - diag_t)}")
                diag_n = n
                diag_t = time.time()

            # frame rate control
            now = time.time()
            last_t += 1 / fps
            if now < last_t:
                time.sleep(last_t - now)
            else:
                last_t = now

    except Exception as e:
        logger.exception(e)
    finally:
        m.config.shadow_config.shadow_amplitude = m.shadow_state.shadow_amplitude.tolist()
        m.config.shadow_config.shadow_offset = m.shadow_state.shadow_offset.tolist()
        env.close()
        logger.debug("rfu exited")
        return