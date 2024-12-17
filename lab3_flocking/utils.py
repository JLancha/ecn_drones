from typing import Dict
import numpy as np

PARAMS: Dict[str, float] = {
    "epsilon": 0.1,
    "h": 0.2,
    "r": 1.2*4,
    "r_prime": 1.2*0.8*4,
    "d": 4.,
    "d_prime": 0.8*4, 
    "a": 5,
    "b": 10,
    "c1_alpha": 2,
    "c2_alpha": 2 * np.sqrt(2),
    "c1_beta": 70,
    "c2_beta": 2 * np.sqrt(70),
    "c1_gamma": 7,
    "c2_gamma": 2 * np.sqrt(7)
}

def sigma_norm(z: np.ndarray):
    """
    Returns the sigma norm of z (Equation 8)

    Parameters:
        z (np.ndarray or float):
            z can be a float or a 1D array. In the second case, the function will compute the sigma norm of each element

    Returns:
        sigma (np.ndarray):
            the sigma norm of z 
    """
    epsilon = PARAMS["epsilon"]
    return (1/epsilon) * (np.sqrt(1+epsilon*np.linalg.norm(z,axis=-1, keepdims=True)**2) - 1)

def bump_function(z: float):
    """
    Computes the bump function defined in Equation 10

    Parameters:
        z (np.ndarray or float):
            z can be a float or a 1D array. In the second case, the bump function is applied to each element

    Returns:
        rho (np.ndarray):
            the bump function applied on z. Value between 0 and 1
    """
    rho = np.zeros(z.shape)
    h = PARAMS["h"]
    rho[z<=1] = (1 + np.cos(np.pi*(z[z<=1]-h)/(1-h)))/2
    rho[z<h] = 1
    rho[z<=0] = 0
    return rho

def phi_alpha(z: float):
    """
    Computes the action function phi_alpha befined in Equation 15.
    This function is used in the gradient-based term in the flocking protocol.

    Parameters:
        z (np.ndarray or float):
            z can be a float or a 1D array. In the second case, the bump function is applied to each element

    Returns:
        phi_alpha (np.ndarray):
            the function applied on z
    """
    r_alpha = sigma_norm([PARAMS["r"]])
    d_alpha = sigma_norm([PARAMS["d"]])
    return bump_function(z/r_alpha) * phi(z-d_alpha)

def phi_beta(z: float):
    """
    Computes the action function phi_beta befined in Equation 56.
    This function is used as a repulsive action in presence of neighboring obstacles.

    Parameters:
        z (np.ndarray or float):
            z can be a float or a 1D array. In the second case, the bump function is applied to each element

    Returns:
        phi_beta (np.ndarray):
            the function applied on z
    """
    r_beta = sigma_norm([PARAMS["r_prime"]])
    d_beta = sigma_norm([PARAMS["d_prime"]])
    return bump_function(z/r_beta) * (sigma_1(z-d_beta)-1)

def sigma_1(z):
    """
    Returns the function sigma_1 of z (Equation 15)

    Parameters:
        z (np.ndarray or float):
            z can be a float or a 1D array. In the second case, the function will compute the sigma norm of each element

    Returns:
        sigma (np.ndarray):
            the function sigma_1 applied on z. 
    """
    return z / np.sqrt(1+z**2)

def phi(z: float):
    """
    Returns the phi function which is a uneven sigmoidal function (Equation 15)

    Parameters:
        z (np.ndarray or float):
            z can be a float or a 1D array. In the second case, the function will compute the sigma norm of each element

    Returns:
        sigma (np.ndarray):
            the function phi applied on z.
    """
    a,b = PARAMS["a"], PARAMS["b"]
    c = np.abs(a-b) / np.sqrt(4*a*b)
    phi = 1/2 * ( (a+b) * sigma_1(z+c) + (a-b) )
    return phi

def compute_aij(qi, qj):
    """
    Return the element (i,j) of the spacial adjacency matrix.
    The adjacency matrix is defined in equation 11.

    Parameters:
        qi (np.ndarray): 
            the position of the robot i
        qj (np.ndarray): 
            position of the robot j.
            (It can be possible to give qj as a 2D matrix of size (number of robots, 2). 
            In this case, the output of the function will be a the vector of size (number of robots,), which is a line of A).

    Returns:
        aij: the element (i, j) of the spacial adjacency matrix
    """
    # TODO: write the formula to compute a_ij based on equation 11.
    z = sigma_norm(qj-qi) / sigma_norm([PARAMS["r"]])
    aij = bump_function(z)
    return aij

def compute_aij_obstacles(qi, qj):
    """
    Return the element (i,j) of the spacial adjacency matrix between robots and obstacles
    The adjacency matrix is defined in equation 11.

    Parameters:
        qi (np.ndarray): 
            the position of the robot i
        qj (np.ndarray): 
            position of obstacle j.
            (It can be possible to give qj as a 2D matrix of size (number of robots, 2). 
            In this case, the output of the function will be a the vector of size (number of robots,), which is a line of A).

    Returns:
        aij: the element (i, j) of the spacial adjacency matrix
    """
    z = sigma_norm(qj-qi) / sigma_norm([PARAMS["r_prime"]])
    aij = bump_function(z)
    return aij

def compute_nij(qi: np.ndarray, qjs:np.ndarray):
    """
    Returns n_ij, the vector along the line connecting qi and qj.
    This function is defined in equation 23.

    Parameters:
        qi (np.ndarray): 
            the position of the robot i
        qj (np.ndarray): 
            position of neighbor j.
            (It can be possible to give qj as a 2D matrix of size (number of neighbors, 2). 
            In this case, the output of the function will be a the vector of size (number of neighbors,)).

    Returns:
        nij (np.ndarray)
    """
    return (qjs - qi)/np.sqrt(1+PARAMS["epsilon"]*np.linalg.norm(qjs-qi,axis=-1, keepdims=True)**2)
