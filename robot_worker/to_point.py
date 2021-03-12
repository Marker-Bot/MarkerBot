import math


kps = 10
kpr = 10


def calculate_to_point(robot_x, robot_y, robot_angle, dist_x, dist_y):

    azimut = math.atan2((dist_y - robot_y), (dist_x - robot_x))
    course_angle = azimut - robot_angle

    goal_vector = math.sqrt((dist_x - robot_x) ** 2 + (dist_y - robot_y)** 2)

    if abs(course_angle) > math.pi:
        course_angle = course_angle - math.copysign(1, course_angle) * 2 * math.pi

    us = kps * goal_vector
    ur = kpr * course_angle
    v1 = us + ur
    v2 = us - ur

    if v1 > 100 or v1 < -100:
        v1 = math.copysign(100, v1)
    if v2 > 100 or v2 < -100:
        v2 = math.copysign(100, v2)

    control = (v1, v2)
    return control
