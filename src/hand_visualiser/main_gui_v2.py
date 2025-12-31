import argparse
import os
import sys
import threading
import time
from datetime import datetime
from queue import Queue

from modules.constants import shadow_join_names

sys.path.append('../api')  # for importing api.python_client
sys.path.append('../api/python_client')  # for importing api.python_client

import polyscope.imgui as psim
import torch
from loguru import logger

from modules.datamodels import MoCapConfig, ImuMsg, PackedWristTrackerMsg
from modules.gui import polyscope_ui_init, polyscope_mano_mesh_init, polyscope_mano_mesh_refresh, polyscope_ui_main, \
    polyscope_handle_events, polyscope_event_daemon, polyscope_register_generic_rigid_bodies, \
    polyscope_update_generic_rigid_bodies
from modules.utils import frame_mano_to_world
from modules.enumerates import MoCapStateEnum, MoCapHandIDEnum, MoCapThreadPluginEnum

from modules.mocap import MoCap

ev_queue: Queue = Queue()


def callback():
    global ev_queue
    m = MoCap()
    psim.Begin('control', True)
    psim.SetWindowSize((600, 1000))

    # Calibration
    ui_ev, ui_ev_triggered = polyscope_ui_main(m)
    if ui_ev_triggered:
        ev_queue.put(ui_ev)

    # bypass if rfu is running
    if m.thread_manager.is_running(MoCapThreadPluginEnum.RFU_CTRL_LOCAL):
        psim.PushItemWidth(100)
        for i, joint_name in zip(range(len(shadow_join_names)), shadow_join_names):
            _, m.shadow_state.shadow_amplitude[i] = psim.SliderFloat(f'{joint_name}_amp', m.shadow_state.shadow_amplitude[i], v_min=-2, v_max=2, )
            psim.SameLine()
            _, m.shadow_state.shadow_offset[i] = psim.SliderFloat(f'{joint_name}', m.shadow_state.shadow_offset[i],  v_min=-60,  v_max=60, )
        psim.PopItemWidth()
        time.sleep(0.1) # yield cpu time
        return

    # Data-driven
    last_frame, idx, err = MoCap().buffer.peek(-1)
    if last_frame is None: return

    wrist_msg: PackedWristTrackerMsg = last_frame[MoCapThreadPluginEnum.WRIST_TRACKER_SDK] if m.wrist_tracker and m.config.show_tracker else None
    mano_pose_debug = None
    if wrist_msg:
        m.wrist_tracker.spike_coordinate_frame_vis.update_model(wrist_msg.tracked_objects[MoCapHandIDEnum.SPIKE])
        polyscope_update_generic_rigid_bodies(wrist_msg)
    for hand in [m.left_hand, m.right_hand]:
        if not hand: continue
        hand_msg: ImuMsg = last_frame[MoCapThreadPluginEnum.from_hand_id(hand.mano_state.hand_id)]
        if hand_msg is not None:
            # Hand pose
            if hand.mano_state.raw2mano_calibrator:
                mano_pose = hand.mano_state.raw2mano_calibrator.get_mano_param(hand_msg.imu_rotation, map_to_world=True)
                mano_pose_debug = hand_msg.imu_rotation[0].inv() * hand_msg.imu_rotation
                hand.mano_state.pose_params_cpt[:] = mano_pose
            else:
                hand.mano_state.pose_params_cpt[0] = torch.Tensor(frame_mano_to_world(hand_msg.imu_rotation[0]).as_rotvec())

        # Tracker
        if wrist_msg and m.config.show_tracker:
            wrist_pose = wrist_msg.tracked_objects[hand.mano_state.hand_id]
            hand.mano_state.wrist_coordinate_frame_vis.update_model(wrist_pose)
            hand.mano_state.tracker_input = wrist_pose

            if hand.mano_state.wrist_calibrator:
                root_rot, root_pos = hand.mano_state.wrist_calibrator(wrist_pose)
                hand.mano_state.pose_params_cpt[0] = torch.zeros(3) # suppress mano wrist
                hand.mano_state.wrist_pos_params_cpt[:] = torch.Tensor(root_pos)
                hand.mano_state.wrist_rot_params_cpt[:] = torch.Tensor(root_rot.as_matrix())
        else:
            hand.mano_state.wrist_pos_params_cpt[:] = torch.zeros(3)
            hand.mano_state.wrist_rot_params_cpt[:] = torch.eye(3)

        if m.config.debug and mano_pose_debug is not None:
            psim.PushItemWidth(100)
            debug_params = mano_pose_debug.as_euler('xyz', degrees=True)
            for i, joint_name in zip(range(16), [f'joint_{str(i)}' for i in range(16)]):
                for j, axis_name in zip(range(3), ['x', 'y', 'z']):
                    _, debug_params[i][j] = psim.SliderFloat(f'{joint_name}_{axis_name}', debug_params[i][j], -180, 180, )
                    if j < 2:
                        psim.SameLine()
            psim.PopItemWidth()
    polyscope_mano_mesh_refresh(m)
    psim.End()


def background_loop():
    m = MoCap()
    while True:
        m.loop()
        if m.current_state.value == MoCapStateEnum.FINALIZED:
            break
        if not m.mocap_is_running():
            break
        time.sleep(1)
    return 0


def entrypoint(argv):
    global ev_queue

    # parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', help='config file', default='config.json')
    args = parser.parse_args(argv)

    # load config from json
    config, err = MoCapConfig.read_from_disk(args.config)
    if err is not None:
        logger.error(f'config file error: {err}')
        return
    if config.debug:
        logger.configure(handlers=[dict(sink=sys.stdout, level='DEBUG', colorize=True, backtrace=True)])
    else:
        logger.configure(handlers=[dict(sink=sys.stdout, level='INFO', colorize=True, backtrace=True)])

    # create the mocap statemachine and run it in a background thread
    from modules.mocap import MoCap
    m = MoCap(config)
    background_thread = threading.Thread(target=background_loop)
    background_thread.start()
    gui_event_thread = threading.Thread(target=polyscope_event_daemon, args=(ev_queue,))
    gui_event_thread.start()

    # launch gui and statemachine
    ps = polyscope_ui_init()
    polyscope_mano_mesh_init(m)
    ps.set_user_callback(callback)
    polyscope_register_generic_rigid_bodies(m)

    # apiserver
    if m.config.enable_api:
        from modules.mocap import run_flask_app
        server_thread = threading.Thread(target=run_flask_app, args=(m.config.api_port,))
        server_thread.daemon = True
        server_thread.start()

    # View the point cloud and mesh we just registered in the 3D UI
    try:
        ps.show()
    except KeyboardInterrupt:
        logger.info("entrypoint got keyboard interrupt")
    finally:
        # shutting down
        logger.info("entrypoint shutting down")
        # shutdown will set shutdown_requested to True, which will stop the background thread
        m.shutdown()
        logger.info("mocap stopped")
        ps.unshow()
        background_thread.join(timeout=5)
        gui_event_thread.join(timeout=5)
        # stop all threads
        m.thread_manager.reset_all()
    # force exit
    logger.info("program terminated")

    # pylint: disable=protected-access
    os._exit(0)


if __name__ == '__main__':
    import sys

    entrypoint(sys.argv[1:])
