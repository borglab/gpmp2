# Python needs to know about gtsam base classes before it can import module classes
# Else will throw cryptic "referenced unknown base type" error.
import gtsam

from gpmp2.gpmp2 import *

__version__ = "0.3.0"
