import os.path as osp
import queue
import time

import open3d as o3d
import numpy as np
import polyscope as ps
import torch
import torch.nn as nn
from manotorch.manolayer import ManoLayer, MANOOutput
from statemachine import StateMachine
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

from modules.datamodels import WristTrackerMsg


# singleton decorator
def singleton(cls):
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)

        return instances[cls]

    return get_instance


_imu_name_idx_map = {
    "imu_0": 0,
    "imu_1": 1,
    "imu_2": 2,
    "imu_3": 3,
    "imu_4": 4,
    "imu_5": 5,
    "imu_6": 6,
    "imu_7": 7,
    "imu_8": 8,
    "imu_9": 9,
    "imu_10": 10,
    "imu_11": 11,
    "imu_12": 12,
    "imu_13": 13,
    "imu_14": 14,
    "imu_15": 15,
}


def convert_frame_to_quaterion_array_in_mano_order(frame: dict) -> np.ndarray:
    quaterion_np = np.zeros((16, 4))
    for dev_id, imu in frame.items():
        idx = _imu_name_idx_map.get(dev_id, None)
        quaterion_np[idx] = [imu['quatX'], imu['quatY'], imu['quatZ'], imu['quatW']]
    return quaterion_np


def convert_optitrack_messages_to_pos_with_quat(optitrack_message: list[dict], tracked_objects_names) -> tuple[dict[str, list[np.ndarray]], dict[str, list[np.ndarray]]]:
    pos_dict = {obj: [] for obj in tracked_objects_names}
    quat_dict = {obj: [] for obj in tracked_objects_names}
    for objs_msg in optitrack_message:
        for obj in objs_msg.keys():
            if obj not in tracked_objects_names:
                continue
            obj_frame = objs_msg[obj]
            pos_ = obj_frame.position
            pos = np.asarray([pos_.x, pos_.y, pos_.z])
            quat_ = obj_frame.rotation
            quat = np.asarray([quat_.w, quat_.x, quat_.y, quat_.z])
            pos_dict[obj].append(pos)
            quat_dict[obj].append(quat)
    return pos_dict, quat_dict


def get_average_rotation(rotations: list[R]):
    """
    Rotations is a list of 3D numpy array, each array is a batch of rotation
    """
    rot_seq_np = np.stack([r.as_quat() for r in rotations], axis=0)
    rot_mean = np.mean(rot_seq_np, axis=0)
    if len(rot_mean.shape) > 1:
        rot_mean /= np.linalg.norm(rot_mean, axis=1)[:, np.newaxis]
    else:
        rot_mean /= np.linalg.norm(rot_mean)
    return R.from_quat(rot_mean)


def get_loss(loss_f: nn.Module, points: torch.Tensor, pose_category: torch.Tensor) -> torch.Tensor:
    """
    :param points: (N, 21, 3) 21 key points of mano hand of N batches
    :param pose_category: (N, 1) N categories.
    :param loss_f: loss function


    :return loss: loss value
    """
    thumb_point = points[:, 4, :]
    oppose_fingertip_index = pose_category * 4
    oppose_fingertip_index = oppose_fingertip_index.view(-1)
    idx = torch.arange(points.shape[0])
    oppose_fingertip_points = points[idx, oppose_fingertip_index]
    return loss_f(thumb_point, oppose_fingertip_points)


def get_optimized_shape_params(
        joints: torch.Tensor,
        finger_category: torch.Tensor,
        hand_category: str,
        iter_num: int = 500,
        learning_rate: float = 0.01,
        regular_term_weight: float = 0.01,
        output_queue: queue.Queue = None,
        compute_device: torch.device = torch.device('cpu')
) -> torch.Tensor:
    """
    :param joints: (N, 16, 3) 16 joints rotation vector (mano model) of N batches
    :param finger_category: (N, 1) N categories. 2 for F2 3 for F3 4 for F4 5 for F5
    :param hand_category: "left" or "right"
    :param iter_num: number of iterations
    :param learning_rate: learning rate
    :param regular_term_weight: weight of regular term, higher->aggressive
    :param output_queue: queue to put the result
    :param compute_device: device to compute

    :return shape_params: (10, ) optimized shape parameter
    """
    assert hand_category in ['left', 'right']
    assert joints.shape[0] == finger_category.shape[0]

    mano_layer = ManoLayer(
        rot_mode="axisang",
        use_pca=False,
        side=hand_category,
        center_idx=None,
        flat_hand_mean=True,
        mano_assets_root="assets/mano",
    ).to(compute_device)
    joints = joints.view(joints.shape[0], -1)
    shape_params = torch.randn((10,), requires_grad=True)
    optimizer = torch.optim.Adam([shape_params], lr=learning_rate)

    loss_f = nn.MSELoss()

    begin_loss = 0.0
    final_loss = 0.0
    with tqdm(range(iter_num)) as pbar:
        for _ in range(iter_num):
            mano_results: MANOOutput = mano_layer(joints.to(compute_device), shape_params.repeat((joints.shape[0], 1)).to(compute_device))
            verts = mano_results.joints
            optimizer.zero_grad()
            loss = get_loss(loss_f, verts, finger_category) + regular_term_weight * torch.linalg.norm(shape_params)
            if _ == 0:
                begin_loss = loss.item()
            final_loss = loss.item()
            loss.backward()
            optimizer.step()

            pbar.set_description(f'Loss: {loss.detach().cpu().numpy()}')
            pbar.update()
    result = shape_params.detach().cpu()
    print(f"After {iter_num} iterations, the loss dropped from {begin_loss} to {final_loss}")
    if output_queue is not None:
        output_queue.put_nowait(result.numpy())
    return result

def mano_rotvec_to_rfu_euler(mano_pose) -> np.ndarray:
    euler = np.zeros((16, 3))  # save
    r_matrix = R.from_rotvec(mano_pose.detach().cpu().numpy()).as_matrix()
    for i in range(16):
        rotation_matrix = r_matrix[i]  # convert to rotation matrix
        # extract euler xyz
        # TODO: optimize
        euler[i, 0] = -np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2])  # 绕x轴的角度
        euler[i, 1] = np.arctan2(-rotation_matrix[0, 1], np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[0, 2] ** 2))  # 绕y轴的角度
        euler[i, 2] = np.arctan2(rotation_matrix[0, 0], rotation_matrix[1, 1])
    return euler

def rot2joint_positions(rot: np.ndarray) -> np.ndarray:
    # (16, 3) -> (24, )
    # TODO: optimize

    read_list = [(0, 0), (0, 1), (13, 0), (13, 1), (14, 0), (14, 1), (15, 0),
                 (0, 2), (7, 0), (7, 1), (8, 1), (9, 1), (10, 0), (10, 1), (11, 1), (12, 1), (4, 0), (4, 1), (5, 1), (6, 1), (1, 0), (1, 1), (2, 1), (3, 1)]

    joint_positions = np.zeros(shape=(24,), dtype=np.float32)

    for m in range(24):
        joint_positions[m] = rot[read_list[m]]

    return joint_positions


def offset_adjust(joint_angles: np.ndarray):
    # wrist
    joint_angles[0] = -joint_angles[0]
    joint_angles[1] = -joint_angles[1]

    # thumb
    # joint_angles[2] = 0  # THJ5

    joint_angles[3] += 30  # THJ4
    joint_angles[4] += 30  # THJ3
    joint_angles[5] += 10  # THJ2
    joint_angles[6] += 0  # THJ1

    # forefinger
    joint_angles[20] -= 25
    # joint_angles[20] = 0

    # mid finger
    joint_angles[16] += 3
    # joint_angles[16] = 0

    # ring finger
    joint_angles[12] *= -1
    joint_angles[12] -= 15
    # joint_angles[12] = 0

    # little finger
    # joint_angles[7] *= 0.85
    joint_angles[7] = 0  # on the palm
    joint_angles[8] *= -1
    joint_angles[8] -= 45
    # joint_angles[8] = 0

    return joint_angles


def progress_statemachine(m: StateMachine, target_state=None):
    """
    If target_state is None, progress to a stable state
    """
    last_state = target_state
    while m.current_state != last_state:
        last_state = m.current_state
        m.loop()
        time.sleep(0.1)


class CoordinateFrame:
    def __init__(self, key: str, scale=0.05):
        """
        Handful coordinate frame for visualization with Polyscope
        """
        self.key = key
        self.T = np.eye(4)
        self.scale = scale
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
        """
        :param T: 4x4 transformation matrix
        """
        self.T = T
        nodes = self.T @ self.nodes_transposed_init
        self.nodes = nodes[:3, :].T

    def register_model(self):
        """
        Register the coordinate frame model to Polyscope, call this function only once
        """
        ps.register_curve_network(f"x_axis_{self.key}", nodes=self.nodes[[0, 1], :], edges=np.array([[0, 1]]), color=(1, 0, 0))
        ps.register_curve_network(f"y_axis_{self.key}", nodes=self.nodes[[0, 2], :], edges=np.array([[0, 1]]), color=(0, 1, 0))
        ps.register_curve_network(f"z_axis_{self.key}", nodes=self.nodes[[0, 3], :], edges=np.array([[0, 1]]), color=(0, 0, 1))

    def update_model(self, msg: WristTrackerMsg):
        """
        Call this function in user_callback to update model
        """
        if msg is not None:
            T = np.eye(4)
            T[:3, :3] = msg.rot.as_matrix()
            T[:3, 3] = msg.pos
            self.set_transform(T)
        ps.get_curve_network(f"x_axis_{self.key}").update_node_positions(self.nodes[[0, 1], :])
        ps.get_curve_network(f"y_axis_{self.key}").update_node_positions(self.nodes[[0, 2], :])
        ps.get_curve_network(f"z_axis_{self.key}").update_node_positions(self.nodes[[0, 3], :])


_constant_frame_mano_to_world_rot = R.from_euler(seq='xyz', angles=[np.pi / 2, 0, -np.pi]).inv()


def frame_mano_to_world(rotation: R) -> R:
    """
    Convert mano rotation to world, so that a mano model at rest pose is displayed as:
    - middle finger points to +X
    - palm points to -Z
    """
    return _constant_frame_mano_to_world_rot * rotation

def get_nws_points(point_cloud: o3d.geometry.PointCloud):
    vis1 = o3d.visualization.VisualizerWithEditing()
    vis1.create_window('Please press Shift and click 3 Points: Base - North - West. Press Q to exit.')
    render_option = vis1.get_render_option()
    render_option.background_color = (0.2, 0.2, 0.2)
    vis1.add_geometry(point_cloud)
    vis1.update_renderer()
    vis1.run()  # user picks points
    vis1.destroy_window()

    pts_sel = vis1.get_picked_points()
    print(pts_sel)


def get_optimized_shape_params_from_keypoints(
        key_pts: torch.Tensor,
        vert_ids,
        hand_category: str,
        iter_num: int = 500,
        learning_rate: float = 0.01,
        regular_term_weight: float = 0.01,
        compute_device: torch.device = torch.device('cpu')
) -> torch.Tensor:
    """
    :param finger_category: (N, 1) N categories. 2 for F2 3 for F3 4 for F4 5 for F5
    :param hand_category: "left" or "right"
    :param iter_num: number of iterations
    :param learning_rate: learning rate
    :param regular_term_weight: weight of regular term, higher->aggressive
    :param output_queue: queue to put the result
    :param compute_device: device to compute

    :return shape_params: (10, ) optimized shape parameter
    """
    assert hand_category in ['left', 'right']

    mano_layer = ManoLayer(
        rot_mode="axisang",
        use_pca=False,
        side=hand_category,
        center_idx=None,
        flat_hand_mean=True,
        mano_assets_root="assets/mano",
    ).to(compute_device)
    wrist_pos = torch.zeros((3,), requires_grad=True, device=compute_device)
    wrist_rot = torch.zeros((3,), requires_grad=True, device=compute_device)
    zero_tensor = torch.zeros((45,), requires_grad=True, device=compute_device)
    shape_params = torch.randn((10,), requires_grad=True, device=compute_device)
    optimizer = torch.optim.Adam([wrist_pos, wrist_rot, shape_params], lr=learning_rate)

    loss_f = nn.MSELoss()

    key_pts = key_pts.to(compute_device)
    hand_root = torch.mean(key_pts[-2: ], dim = 0)
    key_pts = torch.concat([key_pts[: -2], hand_root.unsqueeze(0)], dim=0)

    begin_loss = 0.0
    final_loss = 0.0
    with tqdm(range(iter_num)) as pbar:
        for _ in range(iter_num):
            rot_params = torch.concat([wrist_rot, zero_tensor], dim=0)
            mano_results: MANOOutput = mano_layer(rot_params.unsqueeze(0), shape_params.unsqueeze(0))
            verts = mano_results.verts.squeeze(0)
            mano_key_pts = verts[vert_ids.to(compute_device)]
            mano_root = torch.mean(verts[-2: ], dim=0)
            mano_key_pts = torch.concat([mano_key_pts[:-2], mano_root.unsqueeze(0)], dim=0)

            optimizer.zero_grad()
            loss = loss_f(mano_key_pts, key_pts) + regular_term_weight * torch.linalg.norm(shape_params)
            if _ == 0:
                begin_loss = loss.item()
            final_loss = loss.item()
            loss.backward()
            optimizer.step()

            pbar.set_description(f'Loss: {loss.detach().cpu().numpy()}')
            pbar.update()
    result = shape_params.detach().cpu()
    print(f"After {iter_num} iterations, the loss dropped from {begin_loss} to {final_loss}")
    return result