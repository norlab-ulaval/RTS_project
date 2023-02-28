import numpy as np
import scipy.linalg

# -----------------------------------------
# utils
# -----------------------------------------


def cot(x):
    """
    Return the cotangent of x

    :param x: angle in radians.
    :type kind: float
    :return: cot(x)
    :rtype: float

    """
    return 1.0 / np.tan(x)


def xxT(x):
    """
    Compute the vector operation :math:`x x^T`

    :param x: input vector.
    :type kind: numpy.ndarray
    :return: :math:`x x^T`
    :rtype: numpy.ndarray

    """
    m = np.zeros(shape=(len(x), len(x)))
    for i in range(len(m)):
        for j in range(len(m[i])):
            m[i][j] = x[i] * x[j]
    return m


def expm(M):
    """
    Return the matrix exponential of a square skew symetric matrix M

    .. note:: This function is not optimized and is based on taylor series

    :param M: input matrix.
    :type kind: numpy.ndarray
    :return: exp(M)
    :rtype: numpy.ndarray

    """
    if M.shape[0] != M.shape[1]:
        raise ValueError(
            "expm: M should be a square skew-symmetric matrix " + str(M.shape)
        )
    return scipy.linalg.expm(M)


def logm(M):
    """
    Return the matrix natural logarithm of a square matrix M

    .. note:: This function is not optimized and is based on taylor series

    :param M: input matrix.
    :type kind: numpy.ndarray
    :return: log(M)
    :rtype: numpy.ndarray

    """
    if M.shape[0] != M.shape[1]:
        raise ValueError("expm: M should be a square matrix " + str(M.shape))
    return scipy.linalg.logm(M)


# -----------------------------------------
# so3/SO3
# -----------------------------------------


def wedge_so3(e):
    """
    Return the skew symetric matrix :math:`\phi^\wedge\in\mathfrak{so}(3)` from its coordinates vector :math:`\phi`

    :param e: 3x1 coordinates vector to be converted.
    :type kind: numpy.ndarray
    :return: 3x3 skew symetric matrix
    :rtype: numpy.ndarray

    """
    if e.shape != (3,):
        raise ValueError(
            "wedge_so3: e should be a vector of length 3 and not " + str(e.shape)
        )
    return np.array([[0, -e[2], e[1]], [e[2], 0, -e[0]], [-e[1], e[0], 0]])


def vee_so3(M):
    """
    Return the coordinates vector :math:`\phi` such that :math:`M=\phi^\wedge`, where :math:`M\in\mathfrak{so}(3)` is a skew symetric matrix

    :param M: 3x3 skew symetric matrix to be converted.
    :type kind: numpy.ndarray
    :return: 3x1 coordinates vector
    :rtype: numpy.ndarray

    """
    if M.shape != (3, 3):
        raise ValueError("vee_so3: M should be a 3x3 Matrix and not " + str(M.shape))
    return np.array([M[2][1], M[0][2], M[1][0]])


def logm_so3(C):
    """
    Return the matrix logarithm of a rotation matrix :math:`R\in SO(3)`

    .. note:: This function is optimized and its closed-form is exact

    :param C: 3x3 rotation matrix :math:`\in SO(3)`.
    :type kind: numpy.ndarray
    :return: 3x3 skew-symetric matrix :math:`\in\mathfrak{so}(3)`
    :rtype: numpy.ndarray

    """
    if C.shape != (3, 3):
        raise ValueError(
            "logm_so3: C should be a 3x3 rotation matrix and not " + str(M.shape)
        )
    theta = np.arccos((np.trace(C) - 1) / 2.0)
    if theta < 1e-5:
        return np.zeros((3, 3))
    return 0.5 * theta / np.sin(theta) * (C - C.T)


def expm_so3(M):
    """
    Return the matrix exponential of a skew-symetric matrix :math:`M=\phi^\wedge\in \mathfrak{so}(3)`

    .. note:: This function is optimized and its closed-form is exact

    :param M: 3x3 skew-symetric matrix :math:`\in\mathfrak{so}(3)`.
    :type kind: numpy.ndarray
    :return: 3x3 rotation matrix :math:`\in SO(3)`
    :rtype: numpy.ndarray

    """
    if M.shape != (3, 3):
        raise ValueError(
            "expm_so3: M should be a 3x3 skew-symetric matrix and not " + str(M.shape)
        )

    bphi = vee_so3(M)
    phi = scipy.linalg.norm(bphi)
    if phi > 1e-5:
        a = bphi / phi
    else:
        return np.eye(3, 3)

    return (
        np.cos(phi) * np.eye(3, 3)
        + (1 - np.cos(phi)) * xxT(a)
        + np.sin(phi) * wedge_so3(a)
    )


def jac_so3(e):
    """
    Return the left Jacobian of a skew-symetric matrix :math:`M=\phi^\wedge\in \mathfrak{so}(3)`

    .. note:: Refer to Barfoot's book for further explanations about this notion.

    .. note:: This function is optimized and its closed-form is exact

    :param e: 3x1 coordinates vector of :math:`\mathfrak{so}(3)`.
    :type kind: numpy.ndarray
    :return: 3x3 matrix`
    :rtype: numpy.ndarray

    """
    if e.shape != (3,):
        raise ValueError(
            "jac_so3: e should be a vector of length 3 and not " + str(e.shape)
        )

    if scipy.linalg.norm(e) < 1e-3:
        return np.eye(3, 3)

    a = e / scipy.linalg.norm(e)
    phi = scipy.linalg.norm(e)

    return (
        np.sin(phi) / phi * np.eye(3, 3)
        + (1.0 - np.sin(phi) / phi) * xxT(a)
        + (1.0 - np.cos(phi)) / phi * wedge_so3(a)
    )


def jac_inv_so3(e):
    """
    Return the inverse of the left Jacobian of a skew-symetric matrix :math:`M=\phi^\wedge\in \mathfrak{so}(3)`

    .. note:: Refer to Barfoot's book for further explanations about this notion.

    .. note:: This function is optimized and its closed-form is exact

    :param e: 3x1 coordinates vector of :math:`\mathfrak{so}(3)`.
    :type kind: numpy.ndarray
    :return: 3x3 matrix`
    :rtype: numpy.ndarray

    """
    if e.shape != (3,):
        raise ValueError(
            "jac_inv_so3: e should be a vector of length 3 and not " + str(e.shape)
        )

    if scipy.linalg.norm(e) < 1e-3:
        return np.eye(3, 3)

    a = e / scipy.linalg.norm(e)
    phi = scipy.linalg.norm(e)
    return (
        phi / 2.0 * cot(phi / 2.0) * np.eye(3, 3)
        + (1 - phi / 2.0 * cot(phi / 2.0)) * xxT(a)
        - phi / 2.0 * wedge_so3(a)
    )


# -----------------------------------------
# se3/SE3
# -----------------------------------------


def curly_wedge_se3(e):
    """
    Return the matrix :math:`\\xi^\curlywedge\in \\text{ad}(\mathfrak{so}(3))` from its coordinates vector :math:`\\xi`

    .. note:: It is a shortcut operation needed in Jacobians calculations and other advanced stuff

    :param e: 6x1 coordinates vector to be converted.
    :type kind: numpy.ndarray
    :return: 6x6 skew symetric matrix
    :rtype: numpy.ndarray

    """
    if e.shape != (6,):
        raise ValueError(
            "curly_wedge_se3: e should be a vector of length 6 and not " + str(e.shape)
        )

    rho = e[:3]
    phi = e[3:]
    M = np.zeros((6, 6))
    M[:3, :3] = wedge_so3(phi)
    M[3:6, 3:6] = wedge_so3(phi)
    M[:3, 3:6] = wedge_so3(rho)
    return M


def wedge_se3(e):
    """
    Return the matrix :math:`\\xi^\wedge\in\mathfrak{so}(3)` from its coordinates vector :math:`\\xi`

    :param e: 6x1 coordinates vector to be converted.
    :type kind: numpy.ndarray
    :return: 4x4 skew symetric matrix
    :rtype: numpy.ndarray

    """
    if e.shape != (6,):
        raise ValueError(
            "wedge_se3: e should be a vector of length 6 and not " + str(e.shape)
        )

    rho = e[:3]
    phi = e[3:]
    M = np.zeros((4, 4))
    M[:3, :3] = wedge_so3(phi)
    M[:3, 3] = rho
    return M


def vee_se3(M):
    """
    Return the coordinates vector :math:`{\\xi}` such that :math:`M=\\xi^\wedge`, where :math:`M\in\mathfrak{se}(3)`

    :param M: 4x4 matrix to be converted.
    :type kind: numpy.ndarray
    :return: 3x1 coordinates vector
    :rtype: numpy.ndarray

    """
    if M.shape != (4, 4):
        raise ValueError("vee_se3: M should be a 4x4 matrix and not " + str(M.shape))

    return np.hstack([M[:3, 3], vee_so3(M[:3, :3])])


def logm_se3(T):
    """
    Return the matrix logarithm of a rotation matrix :math:`R\in SO(3)`

    .. note:: This function is optimized and its closed-form is exact

    :param T: 4x4 transformation matrix :math:`\in SE(3)`.
    :type kind: numpy.ndarray
    :return: 4x4 matrix :math:`\in\mathfrak{se}(3)`
    :rtype: numpy.ndarray

    """
    if T.shape != (4, 4):
        raise ValueError(
            "logm_se3: T should be a 4x4 transformation matrix and not " + str(T.shape)
        )

    bphi = logm_so3(T[:3, :3])
    rho = jac_inv_so3(vee_so3(bphi)) @ T[:3, 3]

    M = np.zeros((4, 4))
    M[:3, :3] = bphi
    M[:3, 3] = rho

    return M


def expm_se3(M):
    """
    Return the matrix exponential of a skew-symetric matrix :math:`M=\phi^\wedge\in \mathfrak{so}(3)`

    .. note:: This function is optimized and its closed-form is exact

    :param M: 4x4 matrix :math:`\in\mathfrak{se}(3)`.
    :type kind: numpy.ndarray
    :return: 4x4 transformation matrix :math:`\in SO(3)`
    :rtype: numpy.ndarray

    """
    if M.shape != (4, 4):
        raise ValueError("expm_se3: M should be a 4x4 matrix and not " + str(M.shape))

    e = vee_se3(M)
    C = expm_so3(wedge_so3(e[3:]))
    r = jac_so3(e[3:]) @ e[:3]

    T = np.zeros((4, 4))
    T[:3, :3] = C
    T[:3, 3] = r
    T[3, 3] = 1

    return T


def jac_se3(e):
    """
    Return the left Jacobian of a matrix :math:`M=\phi^\wedge\in \mathfrak{se}(3)`

    .. note:: Refer to Barfoot's book for further explanations about this notion.

    .. note:: This function is **an approximation**. A closed form, exact fomula exists but is quite lenghty

    :param e: 4x4 matrix :math:`\in\mathfrak{se}(3)`.
    :type kind: numpy.ndarray
    :return: 4x4 transformation matrix :math:`\in SO(3)`
    :rtype: numpy.ndarray

    """
    if e.shape != (6,):
        raise ValueError(
            "jac_inv_se3: e should be a vector of length 6 and not " + str(e.shape)
        )

    return np.eye(6, 6) + 0.5 * curly_wedge_se3(e)


def jac_inv_se3(e):
    """
    Return the inverse of the left Jacobian of a matrix :math:`M=\phi^\wedge\in \mathfrak{se}(3)`

    .. note:: Refer to Barfoot's book for further explanations about this notion.

    .. note:: This function is **an approximation**. A closed form, exact fomula exists but is quite lenghty

    :param e: 4x4 matrix :math:`\in\mathfrak{se}(3)`.
    :type kind: numpy.ndarray
    :return: 4x4 transformation matrix :math:`\in SO(3)`
    :rtype: numpy.ndarray

    """
    if e.shape != (6,):
        raise ValueError(
            "jac_inv_se3: e should be a vector of length 6 and not " + str(e.shape)
        )

    return np.eye(6, 6) - 0.5 * curly_wedge_se3(e)
