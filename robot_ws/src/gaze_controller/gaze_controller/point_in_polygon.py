from geometry_msgs.msg import Point32
from math import inf
from typing import Protocol


class PointProtocol(Protocol):
    """
    Protocol for typechecking. This type allows any object that has fields x and y of type float.
    """
    x: float
    y: float


def are_line_segments_intersecting(start_point_1: PointProtocol, end_point_1: PointProtocol,
                                   start_point_2: PointProtocol, end_point_2: PointProtocol) -> bool:
    """
    Check if the line segments intersect.

    :param start_point_1: Starting point of the first line segment.
    :param end_point_1: End point of the first line segment.
    :param start_point_2: Start point of the second line segment.
    :param end_point_2: End point of the second line segment.
    :return: True if segments intersect
    """
    segment1_dir = Point32(x=end_point_1.x - start_point_1.x, y=end_point_1.y - start_point_1.y)
    segment2_dir = Point32(x=end_point_2.x - start_point_2.x, y=end_point_2.y - start_point_2.y)

    s1_to_s2_dir = Point32(x=start_point_2.x - start_point_1.x, y=start_point_2.y - start_point_1.y)
    s1_to_e2_dir = Point32(x=end_point_2.x - start_point_1.x, y=end_point_2.y - start_point_1.y)
    e1_to_s2_dir = Point32(x=start_point_2.x - end_point_1.x, y=start_point_2.y - end_point_1.y)

    # Check whether end points of the second segment are on different sides of the line defined by the first segment
    dot_prod_1 = -segment1_dir.y * s1_to_s2_dir.x + segment1_dir.x * s1_to_s2_dir.y
    dot_prod_2 = -segment1_dir.y * s1_to_e2_dir.x + segment1_dir.x * s1_to_e2_dir.y
    if dot_prod_1 * dot_prod_2 > 0:
        return False
    # Check whether end points of the first segment are on different sides of the line defined by the second segment
    dot_prod_1 = -segment2_dir.y * s1_to_s2_dir.x + segment2_dir.x * s1_to_s2_dir.y
    dot_prod_2 = -segment2_dir.y * e1_to_s2_dir.x + segment2_dir.x * e1_to_s2_dir.y
    if dot_prod_1 * dot_prod_2 > 0:
        return False

    return True


def is_inside_polygon(polygon_corners: [Point32], point: Point32) -> bool:
    """
    Check if point is inside the given polygon. On the edge -> inside
    :rtype: bool
    :param polygon_corners: List of Point objects following PointProtocol
    :param point: A Point object following PointProtocol
    :return: True if the point p lies inside the polygon[] with n vertices
    """
    n = len(polygon_corners)

    # There must be at least 3 vertices
    # in polygon
    if n < 3:
        return False

    # Create a point for line segment
    # from p to infinite
    extreme = Point32(x=inf, y=point.y)
    count = i = 0

    while True:
        next_point = (i + 1) % n

        if are_line_segments_intersecting(polygon_corners[i], polygon_corners[next_point], point, extreme):
            count += 1

        i = next_point

        if i == 0:
            break

    # Return true if count is odd, false otherwise
    return count % 2 == 1
