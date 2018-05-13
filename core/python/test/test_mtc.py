#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor.core import PropertyMap
from geometry_msgs.msg import Pose
import unittest

class TestPropertyMap(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestPropertyMap, self).__init__(*args, **kwargs)
		self.props = PropertyMap()

	def _check(self, name, value):
		self.props[name] = value
		self.assertEqual(self.props[name], value)

	def test_assign(self):
		self._check("double", 3.14)
		self._check("long", 42)
		self._check("long", 13)
		self._check("bool", True)
		self._check("bool", False)
		self._check("pose", Pose())


if __name__ == '__main__':
		    unittest.main()
