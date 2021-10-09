#!/usr/bin/env python3
import unittest
from math import inf
from gaze_controller.point_in_polygon import are_line_segments_intersecting, is_inside_polygon


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


class MyTestCase(unittest.TestCase):

    #    D           B
    #   |   \   L  / |      M
    #   |    \    /  J ------ K
    #   |  H -- C  --|-------- I
    #   |        F --|--------- G
    #   E __________ A
    def setUp(self) -> None:
        self.A = Point(5.0, 1.0)
        self.B = Point(5.0, 5.0)
        self.C = Point(3.0, 3.0)
        self.D = Point(1.0, 5.0)
        self.E = Point(1.0, 1.0)
        self.F = Point(4.0, 2.0)
        self.G = Point(inf, 2.0)
        self.H = Point(2.0, 3.0)
        self.I = Point(inf, 3.0)
        self.J = Point(5.0, 4.0)
        self.K = Point(inf, 4.0)
        self.L = Point(3.0, 4.0)
        self.M = Point(6.0, 4.0)

        self.polygon = [self.A, self.B, self.C, self.D, self.E]
        self.polygon2 = [self.A, self.A, self.B, self.C, self.D, self.E]

    def test_segments_intersecting(self):
        self.assertTrue(are_line_segments_intersecting(self.A, self.B, self.F, self.G))

    def test_segments_not_intersecting(self):
        # Same line
        self.assertTrue(are_line_segments_intersecting(self.A, self.B, self.A, self.B))
        self.assertTrue(are_line_segments_intersecting(self.E, self.A, self.A, self.E))
        # Parallel line
        self.assertFalse(are_line_segments_intersecting(self.A, self.B, self.D, self.E))

    def test_segments_not_even_tho_lines_would(self):
        self.assertFalse(are_line_segments_intersecting(self.A, self.B, self.D, self.C))

    def test_point_on_Segment(self):
        self.assertTrue(are_line_segments_intersecting(self.A, self.B, self.E, self.A))
        self.assertTrue(are_line_segments_intersecting(self.J, self.K, self.A, self.B))
        self.assertTrue(are_line_segments_intersecting(self.A, self.B, self.J, self.K))

    def test_point_in_polygon(self):
        self.assertTrue(is_inside_polygon(self.polygon, self.F))
        self.assertTrue(is_inside_polygon(self.polygon, self.E))
        self.assertTrue(is_inside_polygon(self.polygon, self.H))
        self.assertTrue(is_inside_polygon(self.polygon, self.J))
        self.assertFalse(is_inside_polygon(self.polygon, self.M))
        self.assertFalse(is_inside_polygon(self.polygon, self.L))
        self.assertTrue(is_inside_polygon(self.polygon2, self.F))
        self.assertTrue(is_inside_polygon(self.polygon2, self.E))




if __name__ == '__main__':
    unittest.main()
