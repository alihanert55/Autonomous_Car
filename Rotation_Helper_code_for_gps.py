def calculate_angle_to_target(lat1, lon1, lat2, lon2, heading, rotation_angle=130):
    bearing_to_target = calculator(lat1, lon1, lat2, lon2)
    rotated_bearing = (bearing_to_target + rotation_angle) % 360
    angle = (rotated_bearing - heading + 360) % 360 
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle

def calculator(lat1, lon1, lat2, lon2):
    lat1_rad = degrees_to_radians(lat1)
    lon1_rad = degrees_to_radians(lon1)
    lat2_rad = degrees_to_radians(lat2)
    lon2_rad = degrees_to_radians(lon2)

    delta_lon = lon2_rad - lon1_rad
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    bearing_rad = math.atan2(y, x)
    bearing_deg = (radians_to_degrees(bearing_rad) + 360) % 360

    return bearing_deg
def degrees_to_radians(degrees):
    return degrees * math.pi / 180

def radians_to_degrees(radians):
    return radians * 180 / math.pi