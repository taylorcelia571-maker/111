"""Arrival tangent from the actual global plan, without moving any waypoint."""
import math


def arrival_yaw(points, goal, span=0.3):
    if len(points) < 2 or not all(math.isfinite(v) for p in points for v in p): return None
    if math.hypot(points[-1][0]-goal[0],points[-1][1]-goal[1]) > 0.25: return None
    end=points[-1]
    distance=0.0
    previous=end
    for point in reversed(points[:-1]):
        distance += math.hypot(previous[0]-point[0],previous[1]-point[1])
        previous=point
        if distance >= span:
            dx,dy=end[0]-point[0],end[1]-point[1]
            if math.hypot(dx,dy) > 0.05: return math.atan2(dy,dx)
    if distance >= 0.05:
        return math.atan2(end[1]-points[0][1],end[0]-points[0][0])
    return None
