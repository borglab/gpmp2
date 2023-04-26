"""Tests for the `obstacle` module."""

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gpmp2


class TestSignedDistanceField(GtsamTestCase):
    """Unit tests for SignedDistanceField class."""
