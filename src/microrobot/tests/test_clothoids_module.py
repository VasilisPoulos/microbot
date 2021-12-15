#!/usr/bin/env python

PKG='microrobot'
import unittest
import numpy as np
from microrobot.scripts.new_cloth import *

class TestClothoids(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEqual(1, 1, "1!=1")

    def should_have_equal_path_and_direction_len(self):
        s1 = np.array([0., 0., 1.])
        p = np.array([1.2, 0., 1.])
        s2 = np.array([1.3, 1., 1.])
        c1 = Clothoid(s1, p, s2, max_dev=0.01, num_of_points=20)
        self.assertEqual(len(c1.direction), len(c1.path))
        
    #TODO: test if: (1) finish is ok , (2) start is ok, (3) clothoid curve has enough 
    # points, (4) path has no duplicate points.

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_clothoids_module', TestClothoids)
