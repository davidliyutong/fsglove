import numpy as np
import open3d as o3d
import polyscope as ps
import polyscope.imgui as psim

from queue import Queue
from loguru import logger

from modules.mocap import MoCap
from modules.utils import CoordinateFrame
from manotorch.manolayer import MANOOutput
from modules.enumerates import MoCapCalibrateInstallationType, MoCapCalibrateShapeType, MoCapHandIDEnum, \
    MoCapCalibrateRootType, MoCapCalibrateWristType, MoCapCalibrateShapeKeyPointType


def polyscope_ui_init():
    """
    Initialize polyscope with default settings
    """
    # Initialize polyscope
    ps.init()

    # Configure polyscope visualize options
    ps.set_up_dir("z_up")
    ps.set_front_dir("neg_x_front")
    ps.set_autocenter_structures(False)

    # Use curve network to visualize the 3 axis
    ps.register_curve_network("x_axis", nodes=np.array([[0, 0, 0], [0.3, 0, 0]]), edges=np.array([[0, 1]]),
                              color=(1, 0, 0))
    ps.register_curve_network("y_axis", nodes=np.array([[0, 0, 0], [0, 0.3, 0]]), edges=np.array([[0, 1]]),
                              color=(0, 1, 0))
    ps.register_curve_network("z_axis", nodes=np.array([[0, 0, 0], [0, 0, 0.3]]), edges=np.array([[0, 1]]),
                              color=(0, 0, 1))

    return ps


def polyscope_mano_mesh_refresh(mocap: MoCap) -> (np.ndarray, np.ndarray):
    """
    Generate mano shape from mano

    state: ManoState
        The state contains everything needed to visualize mano hand

    Returns: (vertices[N_v, 3], faces[N_f, 3])
        Combination of vertices and faces
    """
    for hand in [mocap.left_hand, mocap.right_hand]:
        if hand:
            hand.mano_state.cache_mano_output: MANOOutput = hand.mano_state.mano_layer(
                hand.mano_state.pose_params_cpt.reshape(-1).unsqueeze(0),
                hand.mano_state.shape_params_cpt.unsqueeze(0)
            )
            hand_vertices = hand.mano_state.cache_mano_output.verts.squeeze(0).T  # (3, NV)
            hand_vertices = (hand.mano_state.wrist_rot_params_cpt @ hand_vertices).T
            hand_vertices += hand.mano_state.wrist_pos_params_cpt
            hand.mano_state.mano_mesh_vis.update_vertex_positions(hand_vertices.detach().cpu().numpy())


def polyscope_mano_mesh_init(mocap: MoCap):
    """
    Register a mano state to polyscope, with identifier

    state: ManoState
    """
    if mocap.wrist_tracker:
        mocap.wrist_tracker.spike_coordinate_frame_vis.register_model()

    for hand in [mocap.left_hand, mocap.right_hand]:
        if hand:
            mano_output: MANOOutput = hand.mano_state.mano_layer(
                hand.mano_state.pose_params_cpt.reshape(-1).unsqueeze(0),
                hand.mano_state.shape_params_cpt.unsqueeze(0)
            )
            hand_vertices = mano_output.verts.squeeze(0)  # (NV, 3)
            hand_faces = hand.mano_state.mano_layer.th_faces  # (NF, 3)
            vertices, faces = hand_vertices.detach().cpu().numpy(), hand_faces.detach().cpu().numpy()
            hand.mano_state.mano_mesh_vis = ps.register_surface_mesh(hand.mano_state.identifier, vertices, faces)
            hand.mano_state.mano_mesh_vis.set_color((1, 1, 1))
            if mocap.config.wrist_tracker_cfg:
                hand.mano_state.wrist_coordinate_frame_vis = CoordinateFrame(key=hand.mano_state.hand_id.value)
                hand.mano_state.wrist_coordinate_frame_vis.register_model()


from modules.datamodels import MoCapGUIEvents, WristTrackerMsg, PackedWristTrackerMsg

_current_wrist_sel: int = 0


def polyscope_ui_main(m: MoCap) -> (MoCapGUIEvents, bool):
    """
    main polyscope ui

    Returns: (MoCapEvents, bool)
        MoCapEvents: The events that are triggered
        bool: True if the events are triggered
    """
    global _current_wrist_sel
    ev: MoCapGUIEvents = MoCapGUIEvents()

    psim.Separator()
    psim.Text(f"General")
    ev.mocap_save_config_triggered = psim.Button("Save Config")
    psim.SameLine()
    ev.mocap_start_recording_triggered = psim.Button("Recording.Start")
    psim.SameLine()
    ev.mocap_stop_recording_triggered = psim.Button("Recording.Stop")
    psim.Text(f"status: {'on' if m.is_recording else 'off'}")

    _, m.config.debug = psim.Checkbox("Debug", m.config.debug)
    _, m.config.show_tracker = psim.Checkbox("Show Tracker", m.config.show_tracker)
    _, m.config.enable_control = psim.Checkbox("Enable Control", m.config.enable_control)
    psim.Separator()

    # [genera]
    flag = False
    for hand_id in [MoCapHandIDEnum.LEFT, MoCapHandIDEnum.RIGHT]:
        if m.get_hand_by_id(hand_id) is None:
            continue

        # [genera][left|right]
        psim.Text(f"{hand_id.value} Hand")

        psim.Text("Calibrate Root")
        for i, calib_type in enumerate(MoCapCalibrateRootType):
            # [genera][left|right][root]
            ev.mocap_calibrate_root_triggered = psim.Button(
                f"R_{calib_type.value}") or ev.mocap_calibrate_root_triggered
            if ev.mocap_calibrate_root_triggered:
                ev.mocap_calibrate_root_type = calib_type
                ev.mocap_calibrate_root_target = hand_id
                flag = True
                break
            if i < len(MoCapCalibrateRootType) - 1:
                psim.SameLine()
        if flag:
            break

        psim.Text("Calibrate Installation")
        for i, calib_type in enumerate(MoCapCalibrateInstallationType):
            # [genera][left|right][installation]
            ev.mocap_calibrate_installation_triggered = psim.Button(
                f"I_{calib_type.value}") or ev.mocap_calibrate_installation_triggered
            if ev.mocap_calibrate_installation_triggered:
                ev.mocap_calibrate_installation_type = calib_type
                ev.mocap_calibrate_installation_target = hand_id
                flag = True
                break
            if i < len(MoCapCalibrateInstallationType) - 1:
                psim.SameLine()
        if flag:
            break

        psim.Text("Calibrate Shape")
        for i, calib_type in enumerate(MoCapCalibrateShapeType):
            # [genera][left|right][shape]
            ev.mocap_calibrate_shape_triggered = psim.Button(
                f"S_{calib_type.value}") or ev.mocap_calibrate_shape_triggered
            if ev.mocap_calibrate_shape_triggered:
                ev.mocap_calibrate_shape_type = calib_type
                ev.mocap_calibrate_shape_target = hand_id
                flag = True
                break
            if i < len(MoCapCalibrateShapeType) - 1:
                psim.SameLine()
        if flag:
            break
        psim.Separator()
        for i, keypoint in enumerate(MoCapCalibrateShapeKeyPointType):
            # [genera][left|right][shape][keypoint]
            (keypoint_name, keypoint_index) = keypoint
            ev.mocap_calibrate_shape_keypoint_triggered = psim.Button(
                f"{keypoint_name}") or ev.mocap_calibrate_shape_keypoint_triggered
            if ev.mocap_calibrate_shape_keypoint_triggered:
                ev.mocap_calibrate_shape_keypoint_type = keypoint
                ev.mocap_calibrate_shape_keypoint_target = hand_id
                flag = True
                break
            if keypoint not in [
                MoCapCalibrateShapeKeyPointType.T_TIP, MoCapCalibrateShapeKeyPointType.I_TIP,
                MoCapCalibrateShapeKeyPointType.M_TIP, MoCapCalibrateShapeKeyPointType.R_TIP,
                MoCapCalibrateShapeKeyPointType.L_TIP,
            ]:
                psim.SameLine()
        if flag:
            break
        psim.NewLine()

        psim.Text(f"Calibrate Wrist: keyboard_sel={_current_wrist_sel}")
        if m.config.debug:
            tracker_input = m.get_hand_by_id(hand_id).mano_state.tracker_input
            if tracker_input is not None:
                psim.Text(
                    f"x={round(tracker_input.pos[0], 2)},y={round(tracker_input.pos[1], 2)},z={round(tracker_input.pos[2], 2)}")

        if (psim.IsKeyPressed(psim.ImGuiKey_1)):  # TODO: use a configurable
            logger.info(f"W_{ev.mocap_calibrate_wrist_type} is pressed")
            mocap_calibrate_wrist_type_list = list(MoCapCalibrateWristType)
            ev.mocap_calibrate_wrist_type = mocap_calibrate_wrist_type_list[_current_wrist_sel]
            ev.mocap_calibrate_wrist_target = hand_id
            _current_wrist_sel = (_current_wrist_sel + 1) % len(mocap_calibrate_wrist_type_list)
            break

        for i, calib_type in enumerate(MoCapCalibrateWristType):
            # [genera][left|right][wrist]
            ev.mocap_calibrate_wrist_triggered = psim.Button(
                f"W_{calib_type.value}") or ev.mocap_calibrate_wrist_triggered
            if ev.mocap_calibrate_wrist_triggered:
                ev.mocap_calibrate_wrist_type = calib_type
                ev.mocap_calibrate_wrist_target = hand_id
                flag = True
                break
            if i < len(MoCapCalibrateWristType) - 1:
                psim.SameLine()
        if flag:
            break

        # [genera][left|right][rfu.ctrl]
        psim.Text("RFU.Control")
        ev.mocap_rfu_ctrl_launch_triggered = psim.Button("launch") or ev.mocap_rfu_ctrl_launch_triggered
        if ev.mocap_rfu_ctrl_launch_triggered:
            ev.mocap_rfu_ctrl_launch_target = hand_id
            break
        psim.SameLine()
        ev.mocap_rfu_ctrl_termination_triggered = psim.Button("terminate") or ev.mocap_rfu_ctrl_termination_triggered
        if ev.mocap_rfu_ctrl_termination_triggered:
            break

        # [genera][left|right][rfu.retarget]
        psim.Text("RFU.Retarget")
        ev.mocap_rfu_retarget_open_triggered = psim.Button("open") or ev.mocap_rfu_retarget_open_triggered
        if ev.mocap_rfu_retarget_open_triggered:
            ev.mocap_rfu_retarget_open_target = hand_id
            break
        psim.SameLine()
        ev.mocap_rfu_retarget_save_triggered = psim.Button("save") or ev.mocap_rfu_retarget_save_triggered
        if ev.mocap_rfu_retarget_save_triggered:
            ev.mocap_rfu_retarget_open_target = hand_id
            break
        psim.SameLine()
        ev.mocap_rfu_retarget_close_triggered = psim.Button("close") or ev.mocap_rfu_retarget_close_triggered
        if ev.mocap_rfu_retarget_close_triggered:
            break

        psim.NewLine()
        psim.Separator()

    return ev, hash(ev) != hash(MoCapGUIEvents())


def polyscope_handle_events(ev: MoCapGUIEvents):
    logger.info(f"handling gui events")
    m = MoCap()

    # case: exit
    if ev.mocap_exit_requested:
        # Currently no plan to implement this, as user can click the close button
        pass

    # case: save config
    if ev.mocap_save_config_triggered:
        m.config.save_to_disk()

    # case: calibrate root
    if ev.mocap_calibrate_root_triggered:
        target = ev.mocap_calibrate_root_target
        calib_type = ev.mocap_calibrate_root_type
        m.update_root_calibrator(target, calib_type)

    # case: calibrate installation
    if ev.mocap_calibrate_installation_triggered:
        target = ev.mocap_calibrate_installation_target
        calib_type = ev.mocap_calibrate_installation_type
        m.update_installation_calibrator(target, calib_type)

    # case: calibrate shape
    if ev.mocap_calibrate_shape_triggered:
        target = ev.mocap_calibrate_shape_target
        calib_type = ev.mocap_calibrate_shape_type
        m.update_shape_calibrator(target, calib_type)

    if ev.mocap_calibrate_shape_keypoint_triggered:
        target = ev.mocap_calibrate_shape_keypoint_target
        calib_type = ev.mocap_calibrate_shape_keypoint_type
        m.update_shape_keypoint_calibrator(target, calib_type)

    if ev.mocap_calibrate_wrist_triggered:
        target = ev.mocap_calibrate_wrist_target
        calib_type = ev.mocap_calibrate_wrist_type
        m.update_wrist_calibrator(target, calib_type)
        pass

    # case: launch rfu
    if ev.mocap_rfu_ctrl_launch_triggered:
        target = ev.mocap_rfu_ctrl_launch_target
        m.launch_rfu_control(target)
    if ev.mocap_rfu_ctrl_termination_triggered:
        m.terminate_rfu_control()

    if ev.mocap_rfu_retarget_open_triggered:
        target = ev.mocap_rfu_retarget_open_target
        m.open_rfu_retarget(target)
    if ev.mocap_rfu_retarget_save_triggered:
        target = ev.mocap_rfu_retarget_open_target
        m.save_rfu_retarget(target)
    if ev.mocap_rfu_retarget_close_triggered:
        m.close_rfu_retarget()

    # recording operation
    if ev.mocap_start_recording_triggered:
        m.start_recording()
    if ev.mocap_stop_recording_triggered:
        m.stop_recording()


def polyscope_event_daemon(ev_queue: Queue):
    """
    Daemon to handle events from polyscope

    If the event is None, the daemon will stop
    :param ev_queue: Queue
    """
    while True:
        ev = ev_queue.get()
        if ev is None:
            return
        try:
            polyscope_handle_events(ev)
        except Exception as e:
            # must catch all exceptions, otherwise the daemon will stop
            logger.exception(f"polyscope_handle_events error: {e}")
        ev_queue.task_done()


class PolyScopeGenericRigidBody:

    def __init__(self,
                 key: str,
                 scale=0.05,
                 stl_path: str = None,
                 markers_pos: np.ndarray = None):
        self.key = key
        self.T = np.eye(4)
        self.scale = scale
        self.stl_vertices = None
        self.align_matrix = None
        self.stl_path = stl_path
        self.markers_pos = markers_pos
        self.init_nodes = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ]) * self.scale
        self.nodes = np.copy(self.init_nodes)

        self.nodes_transposed_init = np.ones(shape=(4, 4))
        self.nodes_transposed_init[:3, :] = self.nodes.T

    def set_transform(self, T):
        self.T = T
        nodes = self.T @ self.nodes_transposed_init
        self.nodes = nodes[:3, :].T

    def register_model(self):
        # TODO: load an STL model
        if self.stl_path is None:
            ps.register_curve_network(f"x_axis_{self.key}", nodes=self.nodes[[0, 1], :], edges=np.array([[0, 1]]),
                                      color=(1, 0, 0))
            ps.register_curve_network(f"y_axis_{self.key}", nodes=self.nodes[[0, 2], :], edges=np.array([[0, 1]]),
                                      color=(0, 1, 0))
            ps.register_curve_network(f"z_axis_{self.key}", nodes=self.nodes[[0, 3], :], edges=np.array([[0, 1]]),
                                      color=(0, 0, 1))
        else:
            mesh = o3d.io.read_triangle_mesh(self.stl_path)
            vertices = np.asarray(mesh.vertices)
            faces = np.asarray(mesh.triangles)

            voxel_size = max(mesh.get_max_bound() - mesh.get_min_bound()) / 128

            while len(vertices) > 12000:
                mesh = mesh.simplify_vertex_clustering(voxel_size=voxel_size,
                                                       contraction=o3d.geometry.SimplificationContraction.Average)
                vertices = np.asarray(mesh.vertices)
                faces = np.asarray(mesh.triangles)
                logger.info(f"vertice number of object {self.key} is {len(vertices)}")
                voxel_size = voxel_size * 2
            vertices *= 0.001
            ps.register_surface_mesh(f"mesh_{self.key}", vertices=vertices, faces=faces)
            self.stl_vertices = np.copy(vertices)

    def update_model(self, T=None):
        if self.stl_path is None:
            if T is not None:
                self.set_transform(T)
            p_curve = ps.get_curve_network(f"x_axis_{self.key}")
            p_curve.update_node_positions(self.nodes[[0, 1], :])
            p_curve = ps.get_curve_network(f"y_axis_{self.key}")
            p_curve.update_node_positions(self.nodes[[0, 2], :])
            p_curve = ps.get_curve_network(f"z_axis_{self.key}")
            p_curve.update_node_positions(self.nodes[[0, 3], :])
        else:
            if T is None:
                T = np.eye(4)
            if self.align_matrix is not None:
                T = T @ self.align_matrix
            vertices_new = self.stl_vertices @ T[:3, :3].T + T[:3, 3]
            mesh = ps.get_surface_mesh(f"mesh_{self.key}")
            mesh.update_vertex_positions(vertices_new)

    def align_markers(self, markers: np.ndarray, T: np.ndarray):
        if self.align_matrix is not None:
            return
        if np.min(markers) > 1e4:
            self.align_matrix = None
        if np.max(markers) > 1e4:
            return
        from modules.item_tracker.nokov import ItemTracker

        tracker = ItemTracker(self.markers_pos * 0.001)
        _, rot, t = tracker(markers * 0.001)
        T_true = np.eye(4)
        T_true[:3, :3] = rot
        T_true[:3, 3] = t
        self.align_matrix = np.linalg.inv(T) @ T_true
        logger.info(f"The marker alignment of object {self.key} is completed")


generic_rigid_bodies: dict[str, PolyScopeGenericRigidBody] = dict()

stl_id_mapping = {
    "cup": "assets/object_model/cup.stl",
    "utah_teapot": "assets/object_model/utah_teapot.stl",
    "elephant": "assets/object_model/elephant.stl",
    "large_cube": "assets/object_model/cube_large.stl",
    'stanford_rabbit': "assets/object_model/stanford_bunny.stl",
}

from assets.object_model import markers_id_mapping

def polyscope_register_generic_rigid_bodies(m: MoCap):
    global generic_rigid_bodies

    if m.config.wrist_tracker_cfg is not None:
        rigidbody_id_mapping = m.config.wrist_tracker_cfg.rigidbody_id_mapping
        if rigidbody_id_mapping is not None:
            for key in rigidbody_id_mapping:
                if key not in generic_rigid_bodies and key not in [x.value for x in MoCapHandIDEnum]:
                    generic_rigid_bodies[key] = PolyScopeGenericRigidBody(key, stl_path=stl_id_mapping[key],
                                                                          markers_pos=markers_id_mapping[key])
                    generic_rigid_bodies[key].register_model()


def _make_transmat(msg: WristTrackerMsg) -> np.ndarray:
    res = np.eye(4)
    res[:3, :3] = msg.rot.as_matrix()
    res[:3, 3] = msg.pos
    return res


def polyscope_update_generic_rigid_bodies(msg: PackedWristTrackerMsg):
    global generic_rigid_bodies
    for key in generic_rigid_bodies.keys():
        object_msg = msg.tracked_objects.get(key)
        if object_msg is not None:
            generic_rigid_bodies[key].align_markers(object_msg.markers, _make_transmat(object_msg))
            generic_rigid_bodies[key].update_model(_make_transmat(object_msg))
