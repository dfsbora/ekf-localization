import numpy as np

def reshape_z(z, dim_z, ndim):
    """ ensure z is a (dim_z, 1) shaped vector"""

    z = np.atleast_2d(z)
    if z.shape[1] == dim_z:
        z = z.T

    if z.shape != (dim_z, 1):
        raise ValueError('z (shape {}) must be convertible to shape ({}, 1)'.format(z.shape, dim_z))

    if ndim == 1:
        z = z[:, 0]

    if ndim == 0:
        z = z[0, 0]

    return z
