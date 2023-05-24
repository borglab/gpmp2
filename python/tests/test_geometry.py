"""Tests for the `geometry` module."""

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gpmp2


class TestPose2Vector(GtsamTestCase):
    """Test the Pose2Vector class."""

    def test_constructor(self):
        """Test the Pose2Vector constructor."""
        pose = gtsam.Pose2(0, 0, 0)
        c = np.random.rand(3)
        pose2vector = gpmp2.Pose2Vector(pose, c)
        self.assertIsNotNone(pose2vector)

    def test_pose(self):
        """Test the Pose2Vector pose method."""
        pose = gtsam.Pose2(0, 0, 0)
        c = np.random.rand(3)
        pose2vector = gpmp2.Pose2Vector(pose, c)
        self.assertTrue(pose2vector.pose().equals(pose, 1e-9))
        np.testing.assert_equal(pose2vector.configuration(), c)

    def test_configuration(self):
        """Test the Pose2Vector configuration method."""
        pose = gtsam.Pose2(0, 0, 0)
        c = np.random.rand(3)
        pose2vector = gpmp2.Pose2Vector(pose, c)
        np.testing.assert_equal(pose2vector.configuration(), c)