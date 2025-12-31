import threading

import requests
import numpy as np
from loguru import logger
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
from pydantic import BaseModel

from modules.enumerates import MoCapHandIDEnum

class Joints(BaseModel):
    positions: list[float] = [0.0] * 24

def rfu_retarget_trainer(kill_switch: threading.Event, hand_id: MoCapHandIDEnum):
    from modules.mocap import MoCap # avoid circular import
    m = MoCap()
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
    env.ShowArticulationParameter(shadow.id)

    lower_limit = np.asarray(shadow.data['joint_lower_limit'])
    upper_limit = np.asarray(shadow.data['joint_upper_limit'])

    try:
        while True:
            if kill_switch.is_set():
                break
            # loop update
            m.shadow_state.shadow_target_joint_positions = np.clip(
                np.asarray(shadow.data['joint_positions']),
                lower_limit,
                upper_limit
            )
            
            # operate the robot
            if session is not None:
                try:
                    session.post(
                        f"{m.config.shadow_config.endpoint}/joints/", 
                        json=Joints(positions=m.shadow_state.shadow_target_joint_positions.tolist()).model_dump()
                    )
                except Exception as e:
                    logger.warning(e)
            env.step(1)

    except Exception as e:
        logger.exception(e)
    finally:
        env.close()
        logger.debug("rfu exited")
        return
