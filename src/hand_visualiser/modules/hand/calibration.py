import torch
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, RBF
from torch import nn

from tqdm import tqdm
from scipy.optimize import leastsq
from typing import Any
from manotorch.axislayer import AxisLayerFK
from scipy.spatial.transform import Rotation as R
from manotorch.manolayer import ManoLayer, MANOOutput
from pydantic import BaseModel, model_validator, ConfigDict, field_serializer

from modules.datamodels import WristTrackerMsg
from modules.enumerates import MoCapHandIDEnum
from modules.utils import frame_mano_to_world


class WristPoseCalibrator:
    """
    Calibrate the wrist pose by using ICP algorithm.
    """
    def __init__(
        self,
        hand_tracker_rot: R,
        hand_tracker_pos: np.ndarray,
        distal_pos_mano: np.ndarray,
        distal_pos_tracker: np.ndarray
    ):
        assert hand_tracker_pos.shape == (3,) and hand_tracker_rot.as_matrix().shape == (3, 3)
        assert distal_pos_tracker.shape == (5, 3) and distal_pos_mano.shape == (5, 3)
        # for saving purpose
        self._hand_tracker_rot = hand_tracker_rot
        self._hand_tracker_pos = hand_tracker_pos
        self._distal_pos_mano = distal_pos_mano
        self._distal_pos_tracker = distal_pos_tracker

        rigidbody_transmat = self._compose_transmat(hand_tracker_rot, hand_tracker_pos)

        # point match
        pos_mano_mean = np.mean(distal_pos_mano, axis=0)
        pos_tracker_mean = np.mean(distal_pos_tracker, axis=0)

        distal_pos_mano_dc = distal_pos_mano - pos_mano_mean
        distal_pos_tracker_dc = distal_pos_tracker - pos_tracker_mean

        w = distal_pos_mano_dc.T @ distal_pos_tracker_dc

        U, S, Vh = np.linalg.svd(w, full_matrices=True)

        rot = Vh.T @ U.T
        t = pos_tracker_mean - rot @ pos_mano_mean

        tmp_matrix = np.eye(4)
        tmp_matrix[:3, :3] = rot
        tmp_matrix[:3, 3] = t

        self._t_mat_mano_0 = tmp_matrix
        self._t_mat_rigidbody_0 = rigidbody_transmat
        self._t_mat_inv = np.linalg.inv(rigidbody_transmat) @ self._t_mat_mano_0

    @staticmethod
    def _compose_transmat(rot: R, pos: np.ndarray) -> np.ndarray:
        res = np.eye(4)
        res[:3, :3] = rot.as_matrix()
        res[:3, 3] = pos
        return res

    def encode(self):
        return {
            "hand_tracker_rot": self._hand_tracker_rot.as_quat().tolist(),
            "hand_tracker_pos": self._hand_tracker_pos.tolist(),
            "distal_pos_mano": self._distal_pos_mano.tolist(),
            "distal_pos_tracker": self._distal_pos_tracker.tolist()
        }

    @classmethod
    def decode(cls, data: dict):
        return cls(
            hand_tracker_rot=R.from_quat(np.array(data['hand_tracker_rot'])),
            hand_tracker_pos=np.array(data['hand_tracker_pos']),
            distal_pos_mano=np.array(data['distal_pos_mano']),
            distal_pos_tracker=np.array(data['distal_pos_tracker'])
        )

    def get_wrist_transformation_matrix(
        self,
        rigidbody_rot: R,
        rigidbody_pos: np.ndarray,
    ) -> (R, np.ndarray):
        rigidbody_transmat = self._compose_transmat(rigidbody_rot, rigidbody_pos)
        mano_transformation_matrix = rigidbody_transmat @ self._t_mat_inv

        return R.from_matrix(mano_transformation_matrix[:3, :3]), mano_transformation_matrix[:3, 3]

    def __call__(self, msg: WristTrackerMsg) -> (R, np.ndarray):
        return self.get_wrist_transformation_matrix(msg.rot, msg.pos)


class CalibrationQuatSet(BaseModel):
    # TODO: use Rotation
    base_pose: np.ndarray | list[list[float]] | None = None
    z_rot_pose_1: np.ndarray | list[list[float]] | None = None
    z_rot_pose_2: np.ndarray | list[list[float]] | None = None

    model_config = ConfigDict(arbitrary_types_allowed=True)

    @model_validator(mode='before')
    @classmethod
    def check_quat_shape(cls, data: Any) -> Any:
        if isinstance(data, list):
            data = np.asarray(data)
        if isinstance(data, np.ndarray):
            assert data.shape == (16, 4), 'The pose must be of shape (16, 4)'
            assert np.sum((np.linalg.norm(data, axis=1) - np.ones((16,))) ** 2) < 1e-5, 'The norm of imu value must equals to 1'
        return data

    @field_serializer('base_pose', 'z_rot_pose_1', 'z_rot_pose_2')
    def serialize_quat(self, value: np.ndarray) -> np.ndarray:
        return value.tolist()


class Raw2MANOCalibrator:

    def __init__(self,
                 calibration_quat_set: CalibrationQuatSet,
                 side: MoCapHandIDEnum = MoCapHandIDEnum.RIGHT):
        if isinstance(side, str):
            side = MoCapHandIDEnum(side)
        assert side in MoCapHandIDEnum
        self.side = side
        self._axislayer = AxisLayerFK(mano_assets_root="assets/mano", side=self.side.value)
        self._joint_fa = np.asarray(self._axislayer.transf_parent_mapping)
        self._fk_seq = [1, 4, 7, 10, 13, 2, 5, 8, 11, 14, 3, 6, 9, 12, 15]
        self._delta_rot_fa_rotvec = self._get_rest_pose_coord_diff_matrix()

        self._calibration_quat = calibration_quat_set
        self._base_quat = calibration_quat_set.base_pose
        self._global_installation_error = self._get_global_installation_error(calibration_quat_set)
        self._local_installation_error = self._get_local_installation_error()

    def encode(self):
        return {
            "calibration_quat_set": self._calibration_quat.model_dump(),
            "side": self.side,
        }

    @classmethod
    def decode(cls, data: dict):
        return cls(calibration_quat_set=CalibrationQuatSet(**data['calibration_quat_set']), side=data['side'])

    def _get_rest_pose_coord_diff_matrix(self):
        mano_layer = ManoLayer(
            rot_mode="axisang",
            use_pca=False,
            side=self.side.value,
            center_idx=None,
            flat_hand_mean=True,
            mano_assets_root="assets/mano",
        )
        bs = 1
        random_shape = torch.rand(bs, 10)
        hand_pose = torch.zeros(bs, 48)
        mano_results: MANOOutput = mano_layer(hand_pose, random_shape)
        T_g_p = mano_results.transforms_abs
        T_g_a, _, _ = self._axislayer(T_g_p)
        rot_mats = T_g_a[0, :, :3, :3]
        return (R.from_matrix(rot_mats[self._joint_fa]).inv() * R.from_matrix(rot_mats)).as_rotvec()

    def _get_local_installation_error(self) -> np.ndarray:
        return self._get_local_installation_error_by_predefined_pose(np.zeros((16, 3)), np.zeros((16, 3)))

    def _get_local_installation_error_by_predefined_pose(self,
                                                         delta_imu: np.ndarray,
                                                         label_rot: np.ndarray):
        assert delta_imu.shape == (16, 3)
        assert label_rot.shape == (16, 3)

        rotvec_error = np.zeros((16, 3))
        rotvec_mano = np.zeros((16, 3))
        global_inst_error = R.from_rotvec(self._global_installation_error)
        rotvec_mano[0] = (global_inst_error[0] * R.from_rotvec(delta_imu[0]) * global_inst_error[0].inv()).as_rotvec()
        rotvec_error[0] = self._global_installation_error[0]

        for idx in self._fk_seq:
            fa_idx = self._joint_fa[idx]
            pre_rot = R.from_rotvec(rotvec_mano[fa_idx])
            cur_rot = pre_rot * R.from_rotvec(self._delta_rot_fa_rotvec[idx]) * R.from_rotvec(label_rot[idx])
            rotvec_mano[idx] = cur_rot.as_rotvec()
            rotvec_error[idx] = (cur_rot.inv() * R.from_rotvec(self._global_installation_error[idx]) * R.from_rotvec(delta_imu[idx])).as_rotvec()

        return rotvec_error

    def _get_global_installation_error(self, calibration_quat: CalibrationQuatSet) -> np.ndarray:
        base_pose = R.from_quat(calibration_quat.base_pose)
        z_rot_pose_1 = R.from_quat(calibration_quat.z_rot_pose_1)
        z_rot_pose_2 = R.from_quat(calibration_quat.z_rot_pose_2)

        z_rot = (z_rot_pose_2 * z_rot_pose_1.inv()).as_rotvec()
        z_rot = z_rot / np.linalg.norm(z_rot, axis=1)[:, np.newaxis]

        y_axis = np.asarray([0, 0, 1])[np.newaxis, :].repeat(16, 0)
        z_axis = z_rot - y_axis * np.sum(z_rot * y_axis, axis=1)[:, np.newaxis]
        z_axis = z_axis / np.linalg.norm(z_axis, axis=1)[:, np.newaxis]
        if self.side == MoCapHandIDEnum.RIGHT.LEFT:
            z_axis *= z_axis
        x_axis = np.cross(y_axis, z_axis)

        virtual_rot = np.stack([x_axis, y_axis, z_axis], axis=2)
        err_rot = (R.from_matrix(virtual_rot).inv() * base_pose).as_rotvec()
        return err_rot

    def correct_local_inst_error(self,
                                 raw_data: np.ndarray,
                                 pose_rot: np.ndarray):
        """
        :param raw_data: (N, 16, 4) the imu value for N poses
        :param pose_rot: (N, 16, 3) the rotvec predefined for N poses
        """
        mano_layer = ManoLayer(
            rot_mode="axisang",
            use_pca=False,
            side=self.side.value,
            center_idx=None,
            flat_hand_mean=True,
            mano_assets_root="assets/mano",
        )
        bs = pose_rot.shape[0]
        random_shape = torch.rand(bs, 10)
        hand_pose = torch.from_numpy(pose_rot).reshape(bs, -1).float()
        mano_results: MANOOutput = mano_layer(hand_pose, random_shape)
        T_g_p = mano_results.transforms_abs
        _, _, ee = self._axislayer(T_g_p)

        label_rotvec = R.from_euler("XYZ", ee.reshape(-1, 3).numpy()).as_rotvec().reshape(bs, 16, 3)
        local_installation_error_candidate = np.zeros((bs, 16, 3))
        for i in range(bs):
            delta_imu = (R.from_quat(self._base_quat).inv() * R.from_quat(raw_data[i])).as_rotvec()
            local_installation_error_candidate[i] = self._get_local_installation_error_by_predefined_pose(delta_imu, label_rotvec[i])

        quats = (R.from_rotvec(local_installation_error_candidate.reshape(-1, 3)).as_quat()).reshape(bs, 16, 4)
        quats_mean = quats.mean(axis=0)
        quats_mean = quats_mean / np.linalg.norm(quats_mean, axis=1)[:, np.newaxis]
        local_installation_error = R.from_quat(quats_mean).as_rotvec()
        self._local_installation_error = local_installation_error

    def get_mano_param(self, rotations: R, ana_repr: bool = False, map_to_world: bool = False) -> torch.Tensor:
        assert len(rotations) == 16

        rotvec_mano = np.zeros((16, 3))
        rotvec_fk = np.zeros_like(rotvec_mano)

        delta_imu = R.from_quat(self._base_quat).inv() * rotations
        global_inst_error = R.from_rotvec(self._global_installation_error)
        local_inst_error = R.from_rotvec(self._local_installation_error)
        ## compute the root rot
        rotvec_mano[0] = (global_inst_error[0] * delta_imu[0] * local_inst_error[0].inv()).as_rotvec()
        rotvec_fk[0] = R.from_rotvec(rotvec_mano[0]).as_rotvec()

        ## forward kinematic
        for idx in self._fk_seq:
            fa_idx = self._joint_fa[idx]
            pre_rot = R.from_rotvec(rotvec_fk[fa_idx])
            cur_rot = pre_rot * R.from_rotvec(self._delta_rot_fa_rotvec[idx])
            rotvec_mano[idx] = (cur_rot.inv() * global_inst_error[idx] * delta_imu[idx] * local_inst_error[idx].inv()).as_rotvec()
            rotvec_fk[idx] = (cur_rot * R.from_rotvec(rotvec_mano[idx])).as_rotvec()

        if ana_repr:
            return torch.from_numpy(rotvec_mano)

        ana_euler = R.from_rotvec(rotvec_mano).as_euler('XYZ').reshape(1, 16, 3)
        rot_dest_input = self._axislayer.compose(torch.from_numpy(ana_euler).float())[0]

        if map_to_world:
            rot_dest_input[0] = torch.from_numpy(frame_mano_to_world(R.from_rotvec(rotvec_mano[0])).as_rotvec())
        else:
            rot_dest_input[0] = torch.from_numpy(rotvec_mano[0])

        return rot_dest_input

    def __call__(self, rotation: R, ana_repr: bool = False, map_to_world: bool = False) -> torch.Tensor:
        return self.get_mano_param(rotation, ana_repr, map_to_world)


class GPRInstallationCalibrator:
    def __init__(self, train_pose, label_pose):
        assert train_pose.shape == label_pose.shape, "The shape of train_pose and label_pose must be the same"
        assert train_pose.shape[1] == 16, "The second dimension of train_pose and label_pose must be 16 (16 joints)"
        assert train_pose.shape[2] == 3, "The third dimension of train_pose and label_pose must be 3 (rotvec)"

        self._train_pose = train_pose
        self._label_pose = label_pose

        self.gprs = [self._build_gpr([idx]) for idx in range(16)]

    def _build_gpr(self, idx: list[int]):
        kernel = ConstantKernel(constant_value=1.0) * RBF(length_scale=1.0)  # Product of constant and RBF kernels
        gpr = GaussianProcessRegressor(kernel=kernel)  # Create GaussianProcessRegressor object with kernel and noise variance
        train_pose = self._train_pose[:, idx, :]
        target_pose = self._label_pose[:, idx, :]

        gpr.fit(train_pose.reshape(train_pose.shape[0], -1), target_pose.reshape(target_pose.shape[0], -1))  # Fit model to training data
        return gpr

    def get_mano_param(self, mano_params: torch.Tensor) -> torch.Tensor:
        assert mano_params.shape == (16, 3), "The shape of mano_params must be (16, 3)"
        mano_params_np = mano_params.cpu().numpy()
        res = mano_params_np.copy()
        for idx, gpr in enumerate(self.gprs):
            res[idx] = gpr.predict(mano_params_np[idx, :].reshape(1, -1)).reshape(-1, 3)

        return torch.from_numpy(res).to(mano_params.device)


def get_vr_rotation(pose_1: R, pose_2: R ) -> R:
    """
     :param pose_1: the quaternion of pose 1 with shape (4,)
     :param pose_2: the quaternion of pose 2 with shape (4,)
    """

    A_y = (pose_2 * pose_1.inv()).as_rotvec()
    A_y /= np.linalg.norm(A_y)
    A_z = np.array([0., 0., 1.])[np.newaxis, :].repeat(len(pose_1), 0)
    A_x = np.cross(A_y, A_z)

    align_matrix = np.stack([A_x[:,:,np.newaxis], A_y[:,:,np.newaxis], A_z[:,:,np.newaxis]], axis=2).squeeze()

    return R.from_matrix(align_matrix)


class InstallationParams:
    rot_bias: np.ndarray
    rot_amp: np.ndarray

    def __init__(self):
        self.rot_bias = np.zeros((16, 3))
        self.rot_amp = np.ones((16,))


def point_match(src_pts: torch.Tensor, dst_pts: torch.Tensor) -> torch.Tensor:

    src_pts = src_pts.to(dst_pts.device)

    src_mean = torch.mean(src_pts, axis=0)
    dst_mean = torch.mean(dst_pts, axis=0)

    src_dc = src_pts - src_mean
    dst_dc = dst_pts - dst_mean

    w = src_dc.T @ dst_dc

    U, S, Vh = torch.linalg.svd(w, full_matrices=True)

    rot = Vh.T @ U.T
    t = dst_mean - rot @ src_mean

    src_pts = src_pts @ rot.T + t
    return src_pts

keypoints_verts_ids = torch.LongTensor([744, 708, 253,
                                        320, 295, 87, 144,
                                        444, 407, 365, 220,
                                        554, 518, 477, 290,
                                        672, 635, 589, 83,
                                        92, 78])

def get_optimized_shape_params(
        keypoints: torch.Tensor,
        hand_category: str,
        iter_num: int = 500,
        learning_rate: float = 0.01,
        regular_term_weight: float = 0.01,
        compute_device: torch.device = torch.device('cpu')
) -> torch.Tensor:
    """
    :param keypoints: the keypoints tensor of shape (N, 3)
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

    keypoints = keypoints.to(compute_device)
    hand_root = torch.mean(keypoints[-2:], dim = 0)
    keypoints = torch.concat([keypoints[: -2], hand_root.unsqueeze(0)], dim=0)

    begin_loss = 0.0
    final_loss = 0.0
    with tqdm(range(iter_num)) as pbar:
        for _ in range(iter_num):
            rot_params = torch.concat([wrist_rot, zero_tensor], dim=0)
            mano_results: MANOOutput = mano_layer(rot_params.unsqueeze(0), shape_params.unsqueeze(0))
            verts = mano_results.verts.squeeze(0)
            mano_key_pts = verts[keypoints_verts_ids.to(compute_device)]
            mano_root = torch.mean(verts[-2: ], dim=0)
            mano_key_pts = torch.concat([mano_key_pts[:-2], mano_root.unsqueeze(0)], dim=0)

            optimizer.zero_grad()
            loss = loss_f(mano_key_pts, keypoints) + regular_term_weight * torch.linalg.norm(shape_params)
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
