import enum
from typing import Annotated, overload

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
    def __init__(self, arm_preset: str, arm_type: ArmType, dof: int) -> None:
        """create a preset arm solver"""

    def info(self) -> str: ...

    def fk_links(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), device='cpu', writable=False)], arr_rwt: Annotated[NDArray[numpy.float64], dict(shape=(None, None), device='cpu')]) -> None:
        """len(q) >= dof, arr_rwt.shape = (dof+1,7), rwt=[rx,ry,rz,w,tx,ty,tz]"""

    def ik(self, m44_world_tool0: Annotated[NDArray[numpy.float64], dict(shape=(None, None), device='cpu')], arr_qik: Annotated[NDArray[numpy.float64], dict(shape=(None, None), device='cpu')]) -> int:
        """arr_qik.shape = (8,dof)"""

    def set_base(self, m44_world_base: Annotated[NDArray[numpy.float64], dict(shape=(None, None), device='cpu')]) -> None:
        """set pose of arm base in world"""

def transform_xyz(xyz_src: Annotated[NDArray[numpy.float32], dict(shape=(None, 3), device='cpu')], xyz_dst: Annotated[NDArray[numpy.float32], dict(shape=(None, 3), device='cpu')], rwt: Annotated[NDArray[numpy.float32], dict(device='cpu')]) -> None: ...

@overload
def tf44_to_qt7(tf44: Annotated[NDArray[numpy.float32], dict(shape=(4, 4))]) -> Annotated[NDArray[numpy.float32], dict(shape=(7), order='C')]:
    """return qt7"""

@overload
def tf44_to_qt7(tf44: Annotated[NDArray[numpy.float64], dict(shape=(4, 4))]) -> Annotated[NDArray[numpy.float64], dict(shape=(7), order='C')]: ...

@overload
def qt7_to_tf44(qt7: Annotated[NDArray[numpy.float64], dict(shape=(None,))]) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='C')]:
    """return tf44"""

@overload
def qt7_to_tf44(qt7: Annotated[NDArray[numpy.float32], dict(shape=(None,))]) -> Annotated[NDArray[numpy.float32], dict(shape=(4, 4), order='C')]: ...

@overload
def wxyz_t_to_tf44(wxyz: Annotated[NDArray[numpy.float64], dict(shape=(None,))], t: Annotated[NDArray[numpy.float64], dict(shape=(None,))]) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='C')]:
    """return tf44"""

@overload
def wxyz_t_to_tf44(wxyz: Annotated[NDArray[numpy.float32], dict(shape=(None,))], t: Annotated[NDArray[numpy.float32], dict(shape=(None,))]) -> Annotated[NDArray[numpy.float32], dict(shape=(4, 4), order='C')]: ...

def get_stl_data(arg: str, /) -> tuple: ...
