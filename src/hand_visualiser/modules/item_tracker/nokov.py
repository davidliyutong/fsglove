import numpy as np

from itertools import permutations

class ItemTracker:

    def __init__(self, src_pts: np.ndarray):
        self._src_pts = src_pts

    def point_match(self, src_pts: np.ndarray, dst_pts: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:

        src_mean = np.mean(src_pts, axis=0)
        dst_mean = np.mean(dst_pts, axis=0)

        src_dc = src_pts - src_mean
        dst_dc = dst_pts - dst_mean

        w = src_dc.T @ dst_dc

        U, S, Vh = np.linalg.svd(w, full_matrices=True)

        rot = Vh.T @ U.T
        t = dst_mean - rot @ src_mean

        src_pts = src_pts @ rot.T + t
        err = np.linalg.norm(src_pts - dst_pts, axis=1).mean()
        return err, rot, t


    def point_match_random(self,
                   src_pts: np.ndarray,
                   dst_pts: np.ndarray
                   ) -> tuple[float, np.ndarray, np.ndarray]:
        assert src_pts.shape[0] == dst_pts.shape[0]
        perms = permutations(range(src_pts.shape[0]))
        lst_err = 1e10
        bst_rot = None
        bst_t = None
        for perm in perms:
            err, rot, t = self.point_match(src_pts.copy()[np.asarray(perm)], dst_pts)
            if err < lst_err:
                lst_err = err
                bst_rot = rot
                bst_t = t
        return lst_err, bst_rot, bst_t

    def __call__(self, dst_pts: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
        err, rot, t = self.point_match_random(self._src_pts, dst_pts)
        return err, rot, t

    def get_transformed_pcd(self,
                            rot: np.ndarray,
                            t: np.ndarray) -> np.ndarray:
        return self._src_pts @ rot.T + t


