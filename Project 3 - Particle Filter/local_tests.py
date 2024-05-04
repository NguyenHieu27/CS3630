from utils import *
import unittest
from math import isclose
from unittest.mock import patch

class TestGridDistance(unittest.TestCase):

    def test_grid_distance(self):
        result = grid_distance(2, 1, 4, 5)
        self.assertEqual(result, math.sqrt(20))
        print("Good job! Test for grid_distance passed.")

    def test_rotate_point(self):
        x,y = rotate_point(10,10,60)
        self.assertEqual((round(x,2),round(y,2)),(-3.66,13.66))
        print("Good job! Test for rotate_point passed.")

    def test_add_gaussian_noise(self):
        iter=1000
        sum=0
        for i in range(iter):
            sum+=add_gaussian_noise(10,1)
        result=sum/iter
        print(result)
        self.assertAlmostEqual(result,10,delta=0.5)
        print("Good job! Test for add_gaussian_noise passed.")


if __name__ == '__main__':
    unittest.main()
