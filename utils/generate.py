
import csv
import numpy as np


def spherical_to_decart(radius, theta, phi):
    x = radius * np.sin(theta) * np.cos(phi)
    y = radius * np.sin(theta) * np.sin(phi)
    z = radius * np.cos(theta)
    return x, y, z


def decart_to_spherical(x, y, z):
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z/r)
    phi = np.arctan(y / x)
    return r, theta, phi


def draw_random_number_from_pdf(pdf, interval, pdfmax=1, integers=False, max_iterations=10000):
    """
    Draws a random number from given probability density function.

    Parameters
    ----------
        pdf       -- the function pointer to a probability density function of form P = pdf(x)
        interval  -- the resulting random number is restricted to this interval
        pdfmax    -- the maximum of the probability density function
        integers  -- boolean, indicating if the result is desired as integer
        max_iterations -- maximum number of 'tries' to find a combination of random numbers (rand_x, rand_y) located below the function value calc_y = pdf(rand_x).

    returns a single random number according the pdf distribution.
    """
    for i in range(max_iterations):
        if integers == True:
            rand_x = np.random.randint(interval[0], interval[1])
        else:
            # (b - a) * random_sample() + a
            rand_x = (interval[1] - interval[0]) * \
                np.random.random(1) + interval[0]

        rand_y = pdfmax * np.random.random(1)
        calc_y = pdf(rand_x)

        if (rand_y <= calc_y):
            return rand_x

    raise Exception("Could not find a matching random number within pdf in " +
                    max_iterations + " iterations.")


def generate_object_states(object_count=100,
                           orbit_type='circle',
                           orbit_altitude_mean=800000,
                           orbit_altitude_var=200000,
                           object_min_size=0.1,
                           object_max_size=10.0) -> 'list of object parameters':
    """
    Generates list of space junk.
    :param object_count:
    :param orbit_type:
    :param orbit_altitude_mean:
    :param orbit_altitude_var:
    :param object_min_size:
    :param object_max_size:
    :return:
    """

    """Generate orbit position"""
    mu, sigma, size = orbit_altitude_mean, orbit_altitude_var, object_count
    altitudes = np.random.normal(mu, sigma, size)

    thetas = np.random.uniform(0, 2*np.pi, size=object_count)
    phis = np.random.uniform(0, 2*np.pi, size=object_count)

    """Generate object sizes"""
    sizes = [draw_random_number_from_pdf(lambda x: 1 / x, [object_min_size, object_max_size])[0]
             for _ in range(object_count)]

    object_list = []
    for object_idx, altitude, theta, phi, size in zip(
            range(object_count), altitudes, thetas, phis, sizes):
        x, y, z = spherical_to_decart(altitude, theta, phi)
        object_list.append({'time': 0.0,
                            'object_id': object_idx,
                            'x': x,
                            'y': y,
                            'z': z,
                            'size': size})

    return object_list


def generates_objects_in_file(filename,
                              object_count=100,
                              orbit_type='circle',
                              orbit_altitude_mean=800000,
                              orbit_altitude_var=200000,
                              object_min_size=0.1,
                              object_max_size=10.0):
    my_object_list = generate_object_states(object_count, orbit_type, orbit_altitude_mean,
                                            orbit_altitude_var, object_min_size, object_max_size)

    with open(filename, 'w', newline='') as csvfile:
        spamwriter = csv.writer(csvfile, delimiter=' ',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        spamwriter.writerow(['object_id', 'altitude', 'size'])
        for space_object in my_object_list:
            spamwriter.writerow([space_object['object_id'],
                                 space_object['altitude'],
                                 space_object['size']])
    return -1
