import math

from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

def random_point_in_circle(centre,radius):
    # Genera un angolo casuale tra 0 e 2π
        theta = np.random.uniform(0, 2 * math.pi)
        
        # Genera un raggio casuale tra 0 e 1 con distribuzione radiale
        r = math.sqrt(np.random.uniform(0, 1))
        r=r*radius
        
        # Converte le coordinate polari in cartesiane
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        
        return (centre[0]+x, centre[1]+y, 0,701)
def polar_to_euclidian(module,phase,centre):
    # Converte le coordinate polari in cartesiane
    x = module * math.cos(phase)
    y = module * math.sin(phase)
    ret=Point()
    ret.x=centre.x+x
    ret.y=centre.y+y
    ret.z=centre.z
    return ret

def is_segment_in_circles(segment, circles, num_points=100):
    """
    Checks if a line segment is completely contained within an area defined by circles.
    
    Parameters:
    - segment: tuple of ((x1, y1), (x2, y2)) defining the line segment endpoints.
    - circles: list of tuples [(cx, cy, r), ...] defining circles with center (cx, cy) and radius r.
    - num_points: number of points to sample along the segment for the check (default is 100).
    
    Returns:
    - True if the segment is completely contained within at least one of the circles, False otherwise.
    """
    (x1, y1), (x2, y2) = segment
    
    # Generate points along the segment
    for i in range(num_points + 1):
        t = i / num_points
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        
        # Check if this point (x, y) is inside any circle
        point_in_any_circle = False
        for cx, cy, r in circles:
            if (x - cx)**2 + (y - cy)**2 <= r**2:
                point_in_any_circle = True
                break
        
        # If this point is not in any circle, return False
        if not point_in_any_circle:
            return False
    
    # If all points are in at least one circle, return True
    return True
def angle_between_points(p0 : Point, p1 : Point) -> float:

    '''
    Computes the angle between the two given points, in radiants.
    p0: the coordinates of the first point (x0, y0)
    p1: the coordinates of the second point (x1, y1)
    '''

    vector_between = (p1.x - p0.x, p1.y - p0.y)

    norm = math.sqrt(vector_between[0] ** 2 + vector_between[1] ** 2)
    direction = (vector_between[0] / norm, vector_between[1] / norm)

    return math.atan2(direction[0], direction[1]) % (math.pi * 2)


def point_distance(p0 : Point, p1 : Point) -> float:

    vec = (p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)
    return math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)


def move_vector(p0 : Point, p1 : Point) -> tuple[float, float]:

    vec = (math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2), p1.z - p0.z)
    norm = math.sqrt(vec[0]**2 + vec[1]**2)

    return (vec[0]/norm, vec[1]/norm)


def euler_from_quaternion(x : float, y : float, z : float, w : float) -> tuple[float, float, float]:
    
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +1.0 - 2.0 * (y * y + z * z)
    t4 = +2.0 * (w * z + x * y)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def get_yaw(x, y, z, w) -> float:
    return euler_from_quaternion(x,y,z,w)[2] % (math.pi * 2)
