import multiprocessing as mp

from loguru import logger

from modules.enumerates import MoCapProcessPluginEnum
from modules.utils import singleton


@singleton
class MoCapProcessManager:
    """
    MoCapProcessManager manages life cycle of multiprocessing.Process instances.
    """
    def __init__(self):
        self._processes: dict[str, mp.Process | None] = {
            str(k): None for k in list(MoCapProcessPluginEnum)
        }

    def reset(self, ref: MoCapProcessPluginEnum, timeout: float | None = None) -> None:
        if self._processes.get(ref.value) is not None:
            if self._processes[ref.value].is_alive():
                self._processes[ref.value].terminate()
            self._processes[ref.value].join(timeout)
            logger.debug(f"stopped process {ref}")
        self._processes[ref.value] = None

    def reset_all(self, timeout: float | None = None):
        for key in list(MoCapProcessPluginEnum):
            self.reset(MoCapProcessPluginEnum(key), timeout)

    def get_process(self, ref: MoCapProcessPluginEnum) -> mp.Process | None:
        return self._processes.get(ref.value)

    def get_processes(self, ) -> dict[str, mp.Process]:
        return self._processes

    def attach(self, ref: MoCapProcessPluginEnum, t: mp.Process, start: bool = False):
        if start:
            t.start()
        self._processes[ref.value] = t
        logger.debug(f"attached process {ref}, start={start}")

    def is_running(self, ref: MoCapProcessPluginEnum) -> None:
        return self._processes.get(ref.value) is not None and self._processes[ref.value].is_alive()
