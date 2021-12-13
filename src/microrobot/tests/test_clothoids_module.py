#!/usr/bin/env python

PKG='microrobot'
import unittest
from microrobot.scripts import clothoids

class TestClothoids(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEqual(1, 1, "1!=1")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_clothoids_module', TestClothoids)
