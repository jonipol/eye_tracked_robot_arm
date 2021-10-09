from geometry_msgs.msg import Point

from typing import List


def find_corner_points(points: List[Point]) -> (List[Point], Point):
    corner_points = []
    try:
        min_x = min(points, key=lambda i: i.x).x
        max_x = max(points, key=lambda i: i.x).x
        min_y = min(points, key=lambda i: i.y).y
        max_y = max(points, key=lambda i: i.y).y
    except (TypeError, IndexError):
        min_x = max_x = min_y = max_y = float('inf')
    min_point = Point(x=min_x, y=min_y)

    if min_x != float('inf'):
        corner_points.append(Point(x=min_x, y=min_y))
        corner_points.append(Point(x=min_x, y=max_y))
        corner_points.append(Point(x=max_x, y=max_y))
        corner_points.append(Point(x=max_x, y=min_y))

    return corner_points, min_point
