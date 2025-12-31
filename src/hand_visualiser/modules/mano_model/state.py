import torch
import numpy as np
import polyscope as ps
from manotorch.manolayer import ManoLayer, MANOOutput
from scipy.spatial.transform import Rotation as R

from modules.constants import trivial_mano_pose, trivial_mano_tensor, trivial_position
from modules.datamodels import WristTrackerMsg
from modules.enumerates import MoCapHandIDEnum, MoCapCalibrateShapeKeyPointType
from modules.utils import CoordinateFrame
from modules.hand.calibration import Raw2MANOCalibrator, WristPoseCalibrator


class ManoState:
    def __init__(
            self,
            identifier: str = "default",
            hand_id: MoCapHandIDEnum = MoCapHandIDEnum.RIGHT,
            device: torch.device = None
    ):
        # identifier
        self.identifier: str = identifier
        self.hand_id: MoCapHandIDEnum = hand_id

        # cache
        self.cache_mano_output: MANOOutput = None

        # gui controlled euler angles
        self.pose_params_euler = np.zeros(shape=(16, 3))

        # root calibration parameters
        self.calib_root_horizontal_pose = trivial_mano_pose
        self.calib_root_vertical_up_pose = trivial_mano_pose
        self.calib_root_vertical_down_pose = trivial_mano_pose
        self.raw2mano_calibrator: Raw2MANOCalibrator | None = None

        # installation calibration parameters
        self.calib_inst_pose_0 = trivial_mano_pose
        self.calib_inst_pose_1 = trivial_mano_pose
        self.calib_inst_pose_2 = trivial_mano_pose
        self.calib_inst_mano_pose_0 = trivial_mano_tensor  # REST
        self.calib_inst_mano_pose_1 = trivial_mano_tensor  # THUMB UP
        self.calib_inst_mano_pose_2 = trivial_mano_tensor  # FOUR

        # shape calibration parameters
        self.calib_shape_index: list[R] = []
        self.calib_shape_middle: list[R] = []
        self.calib_shape_ring: list[R] = []
        self.calib_shape_little: list[R] = []
        self.calib_shape_keypoints: np.ndarray = np.zeros(shape=(len(MoCapCalibrateShapeKeyPointType), 3))

        # wrist calibration parameters
        self.calib_wrist_thumb_tip_pos = trivial_position
        self.calib_wrist_index_tip_pos = trivial_position
        self.calib_wrist_middle_tip_pos = trivial_position
        self.calib_wrist_ring_tip_pos = trivial_position
        self.calib_wrist_little_tip_pos = trivial_position
        self.wrist_calibrator: WristPoseCalibrator | None = None

        # compute purpose tensors
        self.pose_params_cpt = torch.zeros(16, 3)
        self.shape_params_cpt = torch.zeros(10, )
        self.wrist_pos_params_cpt = torch.zeros(3, )
        self.wrist_rot_params_cpt = torch.eye(3, 3)
        self.tracker_input: WristTrackerMsg = None

        # visualization purpose mesh
        self.mano_layer = ManoLayer(use_pca=False, flat_hand_mean=True)
        self.mano_mesh_vis: ps.SurfaceMesh | None = None
        self.wrist_coordinate_frame_vis: CoordinateFrame | None = None

        # gpu optimization
        if device is None:
            self.compute_device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')
        else:
            self.compute_device = device
        self.mano_layer.to(self.compute_device)
        self.pose_params_cpt = self.pose_params_cpt.to(self.compute_device)
        self.shape_params_cpt = self.shape_params_cpt.to(self.compute_device)
        self.wrist_pos_params_cpt = self.wrist_pos_params_cpt.to(self.compute_device)
        self.wrist_rot_params_cpt = self.wrist_rot_params_cpt.to(self.compute_device)

    def get_distal_pos(self):
        """
        Get the distal joint positions of the MANO model.
        """
        mano_results: MANOOutput = self.mano_layer(torch.zeros(1, 48).to(self.compute_device), self.shape_params_cpt.unsqueeze(0))
        mano_joints = mano_results.joints[0]
        distal_ids = torch.tensor([4, 8, 12, 16, 20])
        mano_distal_pos = mano_joints[distal_ids].cpu().detach().numpy()
        return mano_distal_pos
