import random
import socket
import struct
import time
from typing import List

import numpy as np
import tqdm

ch_imu_data_t_fmt = "I3f3f3f3f4ffI"
hi229_dgram_meta_t_fmt = "=qqIi12s"
ADDR = 0xe5

test_endpoint = "127.0.0.1"
test_port = 18888
block_size = 32
rate = 100
global_start_t = time.time_ns()
device_ids = (
    b'000000000000',
    b'000000000001',
    b'000000000002',
    b'000000000003',
    b'000000000004'
)
device_counters = [0 for _ in device_ids]


def get_socket(endpoint: str, port) -> socket.socket:
    _s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    _s.connect((endpoint, port))
    return _s


def compute_checksum(buf: bytes) -> bytes:
    arr = np.frombuffer(buf, dtype=np.uint8)
    return int(np.bitwise_xor.reduce(arr) % 255).to_bytes(length=1, byteorder='big')


def get_payload(n: int = 1) -> List[bytes]:
    global_ts = time.time_ns()
    global_tsf = time.time_ns() - global_start_t

    res = []
    for i in range(len(device_ids)):
        _res = []
        for seq in range(n):
            _packet = (b'\xe5' +
                       struct.pack(ch_imu_data_t_fmt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) +
                       struct.pack(hi229_dgram_meta_t_fmt,
                                   int(seq / rate * 1e9) + global_ts,
                                   int(seq / rate * 1e9) + global_tsf,
                                   device_counters[i], 0,
                                   device_ids[i]))
            _checksum = compute_checksum(_packet)
            _res.append(_packet + _checksum)
            device_counters[i] += 1

        random.shuffle(_res)
        res.append(b''.join(_res))

    return res


if __name__ == "__main__":
    s_list = [get_socket(test_endpoint, test_port) for _ in range(len(device_ids))]

    start_t = time.time()
    cnt = 0

    with tqdm.tqdm() as pbar:
        for _ in range(1000):
            try:
                if (time.time() - start_t) * rate < cnt:
                    time.sleep(0.01)
                    continue

                l_list = get_payload(block_size)
                [s.send(l) for s, l in zip(s_list, l_list)]
                cnt += block_size
                pbar.update(block_size)
            except KeyboardInterrupt as e:
                pass
