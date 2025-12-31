import json
import os
import threading
import time
from enum import Enum

import jsonlines
from loguru import logger

from modules import mocap
from modules.ring_buffer import RingBuffer


class MoCapDataStoreType(str, Enum):
    LOCAL = 'local'
    MONGODB = 'mongodb'


def get_data_store_type(path: str) -> MoCapDataStoreType:
    # TODO: impl the method
    return MoCapDataStoreType.LOCAL


def jsonl_writer(kill_switch: threading.Event, in_buf_map: dict[str, RingBuffer]):
    store_type = get_data_store_type(mocap.MoCap().config.data_store)
    if store_type == MoCapDataStoreType.LOCAL:
        dirname = os.path.join(mocap.MoCap().config.data_store, time.strftime("%Y%m%d_%H%M%S"))
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        streams = list(in_buf_map.keys())
        filename_mapping = {stream: os.path.join(dirname, f'{stream}.jsonl') for stream in streams}

        logger.info(f'recording to {dirname}')
        try:
            writer_mapping = {stream: jsonlines.Writer(open(filename_mapping[stream], 'w')) for stream in streams}
        except Exception as e:
            logger.error(e)
            raise e

        # save running config
        m = mocap.MoCap()
        with open(os.path.join(dirname, "running_config.json"), 'w') as f:
            json.dump(m.config.model_dump(), f)
    else:
        raise NotImplementedError

    try:
        while True:
            if kill_switch.is_set():
                raise KeyboardInterrupt

            written = False
            for stream in streams:
                msgs, _ = in_buf_map[stream].pull()
                if len(msgs) <= 0:
                    continue

                writer_mapping[stream].write_all([msg.to_dict() for msg in msgs])
                written = True

            if not written:
                time.sleep(0.1)

    except KeyboardInterrupt as e:
        pass
    except Exception as e:
        logger.exception(e)
    finally:
        logger.info(f'data saved as {dirname}')
        [w.close() for w in writer_mapping.values()]
