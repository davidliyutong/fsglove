import threading
import time

import numpy as np
from loguru import logger
from scipy.spatial.transform import Rotation as R

from modules.datamodels import PackedWristTrackerMsg
from modules.ring_buffer import RingBuffer
from modules.utils import get_average_rotation


def input_synchronizer(
        kill_switch: threading.Event,
        in_buf_map: dict[str, RingBuffer],
        out_buf: RingBuffer,
):
    """
    Synchronize multiple streams, this is a naive implementation, it collects latest state of each stream and push them
    into the output ring buffer at a fixed refresh rate.
    """
    fps = out_buf.fps

    last_t = time.time()

    while True:
        if kill_switch.is_set():
            break

        # FIXME: this is brute force, use timestamp to sync
        # FIXME: the logic ignores error when one stream stops, this is wrong.
        curr_frame = dict()
        for k, in_buf in in_buf_map.items():
            data, index, err = in_buf.peek()
            curr_frame[k] = data

        out_buf.push(curr_frame)

        # refresh rate control
        now = time.time()
        last_t += 1 / fps
        if now < last_t:
            time.sleep(last_t - now)
        else:
            last_t = now


def sample_tracker_pose_frame_from_msg_buffer(in_buf: RingBuffer, count: int = 50, timeout=None) -> list[PackedWristTrackerMsg]:
    """
    :return: list of PackedWristTrackerMsg
    """
    frame, idx, err = in_buf.peek()
    frames = []
    seq = 0
    start_t = time.time()
    while seq < count:
        frame, _, err = in_buf.peek(idx + seq)
        if timeout and time.time() - start_t > timeout:
            raise TimeoutError("Timeout, not enough frames collected")
        if err is None:
            frames.append(frame)
            seq += 1
        else:
            time.sleep(0.1)

    logger.info("pose frames collected, length: {}".format(len(frames)))
    return frames


def sample_imu_quat_from_msg_buffer(in_buf: RingBuffer, count: int, timeout=None) -> list[R]:
    """
    :return: list of Rotation objects
    """
    frame, idx, err = in_buf.peek()
    imu_rotations = []
    seq = 0
    start_t = time.time()
    while seq < count:
        if timeout and time.time() - start_t > timeout:
            raise TimeoutError("Timeout, not enough frames collected")
        frame, _, err = in_buf.peek(idx + seq)
        if err is None and frame is not None:
            imu_rotations.append(frame.imu_rotation)
            seq += 1
        else:
            time.sleep(0.1)
    logger.info("imu_quat frames collected, length: {}".format(len(imu_rotations)))
    return imu_rotations


def get_averaged_hand_pose_input(in_buf: RingBuffer, n: int = 50, timeout=2) -> R:
    """
    Get averaged hand pose from IMU quaternion

    :return R: averaged rotation, (16,)
    """
    rotations = sample_imu_quat_from_msg_buffer(in_buf, n, timeout)
    res = get_average_rotation(rotations)
    return res


def get_seq_hand_pose_input(in_buf: RingBuffer, n: int = 400, timeout=6) -> list[R]:
    """
    Get sequence of hand pose from IMU quaternion

    :return list[R]: list of rotation
    """
    rot_seq = sample_imu_quat_from_msg_buffer(in_buf, n, timeout)
    return rot_seq

def get_averaged_tracker_position_input(in_buf: RingBuffer, hand_id, n: int = 50, timeout=2) -> np.ndarray | None:
    """
    Get averaged tracker position from wrist tracker

    :return np.ndarray: averaged position
    """
    tracker_frames = sample_tracker_pose_frame_from_msg_buffer(in_buf, n, timeout)
    positions = np.array([frame.tracked_objects[hand_id.value].pos for frame in tracker_frames])
    return np.mean(positions, axis=0)

def get_averaged_tracker_rotation_input(in_buf: RingBuffer, hand_id, n: int = 50, timeout=2) -> np.ndarray | None:
    """
    Get averaged tracker rotation from wrist tracker

    :return np.ndarray: averaged rotation (1,)
    """
    tracker_frames = sample_tracker_pose_frame_from_msg_buffer(in_buf, n, timeout)
    rotations = [frame.tracked_objects[hand_id.value].rot for frame in tracker_frames]
    return get_average_rotation(rotations)
