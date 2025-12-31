import math
import threading
from copy import deepcopy
from functools import reduce
from queue import Queue
from typing import Optional, Any, List, Dict, Tuple


class RingBuffer:
    def __init__(self, size: int = 1024, fps: int = 30):
        self.size = size
        self.head = 1
        self.counter = 1
        self.mutex = threading.Lock()
        self.buf = [None] * size
        self.fps = fps

        self.last_rd_cnt = {'default': 0}

    def reset(self) -> Optional[Exception]:
        with self.mutex:
            self.counter = 1
            self.head = 1
            self.last_rd_cnt = {'default': 0}
            return None

    def push(self, item: Any) -> Optional[Exception]:
        """
        Push an item into the buffer.
        """
        with self.mutex:
            self.buf[self.head] = item
            self.head = (self.head + 1) % self.size
            self.counter += 1
            return None

    def peek(self, index: int = -1) -> Tuple[Optional[Any], int, Optional[Exception]]:
        """
        Peek at the item at the given index.
        """
        if index >= self.counter:
            return None, -1, Exception("index out of range")
        if index < 0 or index <= (self.counter - self.size):
            index = self.counter - 1
        place = index % self.size
        return self.buf[place], index, None

    def pull(self, client_id: str = 'default') -> Tuple[List[Any], Optional[Exception]]:
        """
        Return all unread items in the buffer.
        """
        if client_id not in self.last_rd_cnt.keys():
            self.last_rd_cnt[client_id] = 0

        with self.mutex:
            if self.counter <= self.last_rd_cnt[client_id] + 1:
                return [], None
            if self.last_rd_cnt[client_id] < 0 or self.last_rd_cnt[client_id] <= (self.counter - self.size):
                self.last_rd_cnt[client_id] = self.counter - 1
            ret = []
            for idx in range(self.last_rd_cnt[client_id] + 1, self.counter):
                item, _, err = self.peek(idx)
                if err is not None:
                    return [], err
                ret.append(item)
            self.last_rd_cnt[client_id] = self.counter - 1
            return ret, None

    def get_size(self) -> int:
        return self.size

    def get_counter(self) -> int:
        return self.counter


class RingBufferSynchronizer:
    def __init__(self,
                 stream_ids: List[str],
                 stream_fps: List[int], max_delay_ms: int = 50,
                 max_sync_window_ms: int = 3000):
        self.start_timestamp: Optional[float] = None
        self.latest_timestamp: float = 0

        self.stream_ids = stream_ids
        self.fps = int(reduce(math.lcm, stream_fps))
        self.abs_head_unit_idx: Dict[str, int] = {stream_id: 0 for stream_id in stream_ids}
        self.latest_ready_unit_idx: int = -1

        self.max_delay_ms = max_delay_ms
        self.max_sync_window_ms = max_sync_window_ms
        self.slot_delay = 1 / self.fps
        self.unit_size = int(max_delay_ms / (self.slot_delay * 1000))
        self.unit_delay = self.unit_size * self.slot_delay
        self.unit_num = int(max_sync_window_ms / self.unit_delay / 1000) + 1
        self.ring_delay = self.unit_num * self.unit_delay

        self._empty_unit = [None for _ in range(self.unit_size)]
        self.buffers = {k: [deepcopy(self._empty_unit) for _ in range(self.unit_num)] for k in stream_ids}

        self.ready_unit_queue = Queue()
        self.mutex = threading.Lock()

    def __repr__(self):
        return (f"RingBufferSynchronizer(\n"
                f"  stream_ids={self.stream_ids}, \n"
                f"  fps={self.fps}, \n"
                f"  max_delay_ms={self.max_delay_ms}, \n"
                f"  max_sync_window_ms={self.max_sync_window_ms} \n"
                f"  unit_size={self.unit_size}, \n"
                f"  unit_delay={self.unit_delay}, \n"
                f"  unit_num={self.unit_num}, \n"
                f")")

    def add_packet(self, stream_id: str, timestamp: float, packet: Any) -> Optional[Exception]:
        with self.mutex:
            if stream_id not in self.stream_ids:
                raise ValueError(f"stream_id {stream_id} not in stream_ids {self.stream_ids}")

            # only set start_timestamp once
            if self.start_timestamp is None:
                self.start_timestamp = timestamp

            self.latest_timestamp = max(self.latest_timestamp, timestamp)
            if self.latest_timestamp - timestamp > self.max_sync_window_ms * 1000:
                return Exception("timestamp out of sync window")

            # check if oldest unit is committed
            oldest_abs_head_unit_idx = min(self.abs_head_unit_idx.values())
            if self.abs_head_unit_idx[stream_id] - oldest_abs_head_unit_idx >= self.unit_num:
                return Exception("unit buffer overflow")

            # find a place to put the packet
            abs_unit_idx, offset = divmod((timestamp - self.start_timestamp), self.unit_delay)
            abs_unit_idx = int(abs_unit_idx)
            unit_idx = abs_unit_idx % self.unit_num
            slot_idx = int(offset // self.slot_delay)
            self.buffers[stream_id][unit_idx][slot_idx] = packet
            # update abs_head_unit_idx
            self.abs_head_unit_idx[stream_id] = max(self.abs_head_unit_idx[stream_id], abs_unit_idx)

            # check if oldest unit is committed
            oldest_abs_head_unit_idx = min(self.abs_head_unit_idx.values())
            for abs_idx in range(self.latest_ready_unit_idx + 1, oldest_abs_head_unit_idx):
                rel_idx = abs_idx % self.unit_num
                self.ready_unit_queue.put({k: self.buffers[k][rel_idx] for k in self.stream_ids})
                for k in self.stream_ids:
                    self.buffers[k][rel_idx] = deepcopy(self._empty_unit)
            self.latest_ready_unit_idx = oldest_abs_head_unit_idx - 1

    def get_ready_packets(self, timeout: Optional[float] = None) -> List[Dict[str, List[Any]]]:
        return self.ready_unit_queue.get(timeout=timeout)

    @property
    def queue_ready(self):
        return not self.ready_unit_queue.empty()


def _test():
    # mock test
    stream_ids = ['h1', 'h2', 'opt']
    stream_fps = [100, 100, 120]
    synchronizer = RingBufferSynchronizer(stream_ids, stream_fps)
    print(synchronizer)

    # generate fake packets
    import random
    import time

    packets = {
        k: [] for k in stream_ids
    }
    print("generating packets...")
    start_timestamp = time.time()
    for stream_id, fps in zip(stream_ids, stream_fps):
        timestamp = start_timestamp
        idx = 0
        while (timestamp - start_timestamp) < 10:
            packet = str(timestamp) + '-' + str(idx) + '-' + str(stream_id)  # packet = timestamp + stream_id
            packets[stream_id].append((timestamp, packet))
            timestamp += (random.random() - 0.5) / 10000 + 1 / fps  # simulate random jitter
            idx += 1
    print("done generating packets")

    # add packets to synchronizer in a thread
    def add_packets(stream_id_: str, fps_: int, packets_: List[Tuple[float, Any]]):
        now = time.time()
        start_t = now
        for ts, pkt in packets_:
            synchronizer.add_packet(stream_id_, ts, pkt)
            time.sleep(now + 1 / fps_ - (time.time()))
            now = now + 1 / fps_
        print(f"stream {stream_id_} done in {time.time() - start_t}")

    for stream_id, fps in zip(stream_ids, stream_fps):
        t = threading.Thread(target=add_packets, args=(stream_id, fps, packets[stream_id]))
        t.start()

    # consume packets from synchronizer in main thread
    start_timestamp = time.time()
    print("simulation started at", start_timestamp)
    n = 0
    fail = 0
    try:
        while True:
            ready_packet = synchronizer.get_ready_packets(timeout=0.2)
            n += 1
    except Exception as e:
        print(e)
    print(time.time() - start_timestamp, n)


if __name__ == '__main__':
    _test()
