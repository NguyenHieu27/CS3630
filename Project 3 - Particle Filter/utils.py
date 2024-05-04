import setting
import random
random.seed(setting.RANDOM_SEED)
import math

# Hieu Nguyen (GTID: 903681705)

def grid_distance(x1, y1, x2, y2):
    """
    
    Calculate the Euclidean distance between two points in a grid world.

    Arguments:
        x1, y1: int
            Coordinates of the first point.
        x2, y2: int
            Coordinates of the second point.

    Returns:
        float
            Euclidean distance between the two points.
    """
    dist = 0
    # TODO: implement here
    # ----------------------------------
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    # ----------------------------------
    return dist


def rotate_point(x, y, heading_deg):
    """
    Rotate a point (x, y) by a given angle in degrees.

    Arguments:
        x, y: float
            Coordinates of the point to be rotated.
        heading_deg: float
            Angle of rotation in degrees.

    Returns:
        Tuple[float, float] (xr, yr)
            Coordinates of the rotated point.
    """
    xr, yr = 0, 0

    # TODO: implement here
    # ----------------------------------
    heading_rad = math.radians(heading_deg)
    # Calculate coordinates of rotated point using matrix
    xr = x * math.cos(heading_rad) - y * math.sin(heading_rad)
    yr = x * math.sin(heading_rad) + y * math.cos(heading_rad)
    # ----------------------------------

    return xr, yr

def add_gaussian_noise(variable, sigma=1.0):
    """
    Add zero-mean Gaussian noise to the input variable.

    Arguments: 
        variable: float
            Input variable to which noise will be added.
        sigma: float
            Standard deviation of the Gaussian noise.

    Returns:
        float
            Variable with added Gaussian noise.
    """
    noisy_variable = 0

    # TODO: implement here
    # ----------------------------------
    # Randomize Gaussian noise
    noise = random.gauss(0, sigma)
    # Calculate noisy variable to return
    # noisy_variable = variable + noise
    # ----------------------------------

    return noise + variable

def diff_heading_deg(heading1, heading2):
    """
    Return the difference between two angles, heading1 - heading2.

    Return value always in range (-180, 180] degrees.
    """
    dh = heading1 - heading2
    while dh > 180:
            dh -= 360
    while dh <= -180:
            dh += 360
    return dh


def compute_mean_pose(particles, confident_dist=1):
    """ 
    Compute the mean pose for all particles.

    (This is not part of the particle filter algorithm but rather an
    addition to show the "best belief" for current pose)
    """
    m_x, m_y, m_count = 0, 0, 0
    # for rotation average
    m_hx, m_hy = 0, 0
    for p in particles:
        m_count += 1
        m_x += p.x
        m_y += p.y
        m_hx += math.sin(math.radians(p.h))
        m_hy += math.cos(math.radians(p.h))

    if m_count == 0:
        return -1, -1, 0, False

    m_x /= m_count
    m_y /= m_count

    # average rotation
    m_hx /= m_count
    m_hy /= m_count
    m_h = math.degrees(math.atan2(m_hx, m_hy))

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if grid_distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_h, m_count > len(particles) * 0.95


