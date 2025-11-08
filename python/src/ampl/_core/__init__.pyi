import enum
from typing import Annotated

import numpy
from numpy.typing import NDArray


def version() -> None:
    """Print version of ampl in terminal."""

class ArmType(enum.Enum):
    Humanoid7 = 2

    Industrial6 = 0

    UR6 = 1

Humanoid7: ArmType = ArmType.Humanoid7

Industrial6: ArmType = ArmType.Industrial6

UR6: ArmType = ArmType.UR6

class ArmBase:
    def __init__(self, arg0: str, arg1: ArmType, arg2: int, /) -> None: ...

    def info(self) -> str: ...

    def fk_qt7(self, arg0: Annotated[NDArray[numpy.float64], dict(shape=(None,), device='cpu', writable=False)], arg1: Annotated[NDArray[numpy.float64], dict(shape=(None, None), device='cpu')], /) -> None: ...

def transform_xyz(xyz_src: Annotated[NDArray[numpy.float32], dict(shape=(None, 3), device='cpu')], xyz_dst: Annotated[NDArray[numpy.float32], dict(shape=(None, 3), device='cpu')], rwt: Annotated[NDArray[numpy.float32], dict(device='cpu')]) -> None: ...
