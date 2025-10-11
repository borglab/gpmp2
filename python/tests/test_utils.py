# pylint: disable=no-name-in-module, import-error, no-member
import unittest

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gpmp2


class TestUtils(GtsamTestCase):
    """Unit tests for utils"""

    def test_PriorFactorPose2Vector(self):
        """Test the PriorFactorPose2Vector class"""
        pose2_vector = gpmp2.Pose2Vector()
        factor = gpmp2.PriorFactorPose2Vector(
            0, pose2_vector, gtsam.noiseModel.Isotropic.Sigma(3, 0.1))
        self.assertIsNotNone(factor)

    def test_insertPose2VectorInValues(self):
        """Test insertPose2VectorInValues function"""
        values = gtsam.Values()
        self.assertEqual(values.size(), 0)
        p = gpmp2.Pose2Vector()
        gpmp2.insertPose2VectorInValues(0, p, values)
        self.assertEqual(values.size(), 1)

    def test_atPose2VectorValues(self):
        """Test atPose2VectorValues function"""
        values = gtsam.Values()
        self.assertEqual(values.size(), 0)
        p = gpmp2.Pose2Vector(gtsam.Pose2(), np.ones(3))
        gpmp2.insertPose2VectorInValues(0, p, values)
        self.assertEqual(values.size(), 1)
        p = gpmp2.atPose2VectorValues(0, values)
        self.assertTrue(p.pose().equals(gtsam.Pose2(), 1e-9))
