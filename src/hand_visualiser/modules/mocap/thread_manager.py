import threading
from threading import Thread, Event

from loguru import logger

from modules.enumerates import MoCapThreadPluginEnum
from modules.utils import singleton


@singleton
class MoCapThreadManager:
    """
    MoCapThreadManager manages life cycle of threading.Thread instances.
    """
    def __init__(self):
        self._switches: dict[str, Event | None] = {
            str(k): None for k in list(MoCapThreadPluginEnum)
        }
        self._threads: dict[str, Thread | None] = {
            str(k): None for k in list(MoCapThreadPluginEnum)
        }

    def reset(self, ref: MoCapThreadPluginEnum, timeout: float | None = None) -> None:
        if self._threads.get(ref.value) is not None:
            self._switches[ref.value].set()
            self._threads[ref.value].join(timeout)
            logger.debug(f"stopped thread {ref}")
        self._switches[ref.value] = threading.Event()
        self._threads[ref.value] = None

    def reset_all(self, timeout: float | None = None):
        for key in list(MoCapThreadPluginEnum):
            self.reset(MoCapThreadPluginEnum(key), timeout)

    def get_switch(self, ref: MoCapThreadPluginEnum) -> threading.Event | None:
        return self._switches.get(ref.value)

    def get_thread(self, ref: MoCapThreadPluginEnum) -> threading.Thread | None:
        return self._threads.get(ref.value)

    def get_threads(self) -> dict[str, threading.Thread]:
        return self._threads

    def attach(self, ref: MoCapThreadPluginEnum, t: threading.Thread, start: bool = False):
        if start:
            t.start()
        self._threads[ref.value] = t
        logger.debug(f"attached thread {ref}, start={start}")

    def is_running(self, ref: MoCapThreadPluginEnum) -> None:
        return self._threads.get(ref.value) is not None and self._threads[ref.value].is_alive()


