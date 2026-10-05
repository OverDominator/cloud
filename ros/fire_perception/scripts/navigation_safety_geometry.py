import math

def swept_rectangle_clear(points, linear, angular, horizon, half_length,
                          half_width, margin, steps=8):
    """Check the swept chassis, including reverse and stationary rotations.

    A return already inside the safety margin may only become less intrusive;
    otherwise a robot touching a wall could never back away from it.
    """
    half_length += margin
    half_width += margin
    if not all(math.isfinite(v) for v in (linear, angular, horizon)):
        return False
    samples = []
    for point in points:
        x, y = point[:2]
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        samples.append((x, y, max(abs(x) - half_length,
                                   abs(y) - half_width)))
    for index in range(1, steps + 1):
        t = horizon * index / steps
        heading = angular * t
        if abs(angular) < 1.0e-6:
            x, y = linear * t, 0.0
        else:
            x = linear * math.sin(heading) / angular
            y = linear * (1.0 - math.cos(heading)) / angular
        cosine, sine = math.cos(heading), math.sin(heading)
        for point_x, point_y, initial in samples:
            dx, dy = point_x - x, point_y - y
            forward = cosine * dx + sine * dy
            left = -sine * dx + cosine * dy
            clearance = max(abs(forward) - half_length,
                            abs(left) - half_width)
            if clearance < min(0.0, initial) - 1.0e-4:
                return False
    return True


def swept_fire_clear(zones, pose, linear, angular, horizon, clearance,
                     steps=8):
    """Never cross a known fire boundary; inside it only allow movement out."""
    px, py, yaw = pose
    for zx, zy, radius in zones:
        initial = math.hypot(px - zx, py - zy)
        lower_bound = min(initial, radius + clearance)
        for index in range(1, steps + 1):
            t = horizon * index / steps
            if abs(angular) < 1.0e-6:
                x, y = linear * t, 0.0
            else:
                x = linear * math.sin(angular * t) / angular
                y = linear * (1.0 - math.cos(angular * t)) / angular
            wx = px + math.cos(yaw) * x - math.sin(yaw) * y
            wy = py + math.sin(yaw) * x + math.cos(yaw) * y
            if math.hypot(wx - zx, wy - zy) < lower_bound - 1.0e-4:
                return False
    return True
