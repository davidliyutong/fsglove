import numpy as np

from modules.datamodels import MoCapShadowConfig


class ShadowState:
    def __init__(self):
        self.shadow_offset = np.zeros(shape=(24,))
        self.shadow_amplitude = np.ones(shape=(24,))
        self.shadow_target_joint_positions = np.zeros(shape=(24,))
        self.endpoint: str | None = None

    @classmethod
    def from_shadow_config(cls, cfg: MoCapShadowConfig | None):
        res = cls()
        if cfg is None:
            return res
        res.shadow_offset = np.array(cfg.shadow_offset)
        res.shadow_amplitude = np.array(cfg.shadow_amplitude)
        res.endpoint = cfg.endpoint
        return res

    def to_shadow_config(self) -> MoCapShadowConfig:
        return MoCapShadowConfig(
            shadow_offset=self.shadow_offset.tolist(),
            shadow_amplitude=self.shadow_amplitude.tolist(),
            endpoint=self.endpoint,
        )


