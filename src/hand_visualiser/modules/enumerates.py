from enum import Enum


class MoCapHandStateEnum(str, Enum):
    UNCONNECTED = "unconnected"
    CONNECTED = "connected"
    EXITED = "exited"


class MoCapTrackerStateEnum(str, Enum):
    UNCONNECTED = "unconnected"
    CONNECTED = "connected"
    EXITED = "exited"


class MoCapStateEnum(str, Enum):
    INITIALIZED = "initialized"
    UNCONNECTED = "unconnected"
    CONNECTED = "connected"
    RUNNING = "running"
    EXITED = "exited"
    FINALIZED = "finalized"


class MoCapThreadPluginEnum(str, Enum):
    SYNC_LOCAL = "sync.local"
    LEFT_HAND_RPC = "left_hand.rpc"
    RIGHT_HAND_RPC = "right_hand.rpc"
    WRIST_TRACKER_SDK = "wrist_tracker.sdk"
    RFU_CTRL_LOCAL = "rfu_control.local"
    RFU_RETARGET_LOCAL = "rfu_retarget.local"
    RECORDING_LOCAL = "recording.local"

    @staticmethod
    def from_hand_id(v):
        if v == MoCapHandIDEnum.RIGHT:
            return MoCapThreadPluginEnum.RIGHT_HAND_RPC
        elif v == MoCapHandIDEnum.LEFT:
            return MoCapThreadPluginEnum.LEFT_HAND_RPC
        else:
            raise ValueError(f"Unknown hand_id: {v}")


class MoCapProcessPluginEnum(str, Enum):
    GUI_LOCAL = "gui.local"
    OPTIMIZE_SHAPE_LOCAL = "optimize.shape.local"


class MoCapCalibrateRootType(str, Enum):
    HORIZONTAL = "horizontal"  #
    VERTICAL_UP = "vertical_up"  #
    VERTICAL_DOWN = "vertical_down"  #
    RESET = "reset"  #
    CALC = "calc"  #


class MoCapCalibrateInstallationType(str, Enum):
    POSE_0 = "pose_0"  # five
    POSE_1 = "pose_1"  # thumb-up
    POSE_2 = "pose_2"  # four
    RESET = "reset"  #
    CALC = "calc"  #


class MoCapCalibrateShapeType(str, Enum):
    INDEX = "index"
    MIDDLE = "middle"
    RING = "ring"
    LITTLE = "little"
    RESET = "reset"
    CALC = "calc"  #


class MoCapCalibrateShapeKeyPointType(tuple[str, int], Enum):
    T_MCP = ("T_MCP", 0)
    T_PIP = ("T_PIP", 1)
    T_DIP = ("T_DIP", 2)
    T_TIP = ("T_TIP", 3)
    I_MCP = ("I_MCP", 4)
    I_PIP = ("I_PIP", 5)
    I_DIP = ("I_DIP", 6)
    I_TIP = ("I_TIP", 7)
    M_MCP = ("M_MCP", 8)
    M_PIP = ("M_PIP", 9)
    M_DIP = ("M_DIP", 10)
    M_TIP = ("M_TIP", 11)
    R_MCP = ("R_MCP", 12)
    R_PIP = ("R_PIP", 13)
    R_DIP = ("R_DIP", 14)
    R_TIP = ("R_TIP", 15)
    L_MCP = ("L_MCP", 16)
    L_PIP = ("L_PIP", 17)
    L_DIP = ("L_DIP", 18)
    L_TIP = ("L_TIP", 19)
    W_LEFT = ("W_LEFT", 20)
    W_RIGHT = ("W_RIGHT", 21)
    RESET = ("reset", None)
    CALC = ("calc", None)



class MoCapCalibrateWristType(str, Enum):
    THUMB = "thumb"
    INDEX = "index"
    MIDDLE = "middle"
    RING = "ring"
    LITTLE = "little"
    RESET = "reset"
    CALC = "calc"


class MoCapHandIDEnum(str, Enum):
    UNKNOWN = 'unknown'
    NONE = 'none'
    LEFT = 'left'
    RIGHT = 'right'
    SPIKE = 'spike'


class MoCapFingerType(str, Enum):
    THUMB = "thumb"
    INDEX = "index"
    MIDDLE = "middle"
    RING = "ring"
    LITTLE = "little"
