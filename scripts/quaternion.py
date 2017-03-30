import math

def rpy2quat(rpy):
    halfroll = rpy[0]/2.0
    halfpitch = rpy[1]/2.0
    halfyaw = rpy[2]/2.0

    sin_r2 = math.sin(halfroll)
    sin_p2 = math.sin(halfpitch)
    sin_y2 = math.sin(halfyaw)

    cos_r2 = math.cos(halfroll)
    cos_p2 = math.cos(halfpitch)
    cos_y2 = math.cos(halfyaw)

    q = []
    q.append(cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2)
    q.append(sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2)
    q.append(cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2)
    q.append(cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2)
    return q

def quat2rpy(q):
    r_a = 2.0 * (q[0]*q[1] + q[2]*q[3])
    r_b = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2])
    roll = math.atan2(r_a, r_b)

    pitch_sin = 2.0 * (q[0]*q[2] - q[3]*q[1])
    pitch= math.asin(pitch_sin)

    y_a = 2.0 * (q[0]*q[3] + q[1]*q[2])
    y_b = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3])
    yaw = math.atan2(y_a, y_b)

    return [roll, pitch, yaw]
