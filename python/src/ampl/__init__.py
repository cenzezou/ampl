"""
AMPL = Another Motion Planning Library
"""
__all__ = [
    "version",
    "ArmBase"
   
]
# from . import _core
# from . import _utils


from ._core import version as version
from ._core import ArmBase as ArmBase
from ._core import ArmType as ArmType