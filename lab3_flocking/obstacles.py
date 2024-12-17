import numpy as np
import matplotlib.pyplot as plt

class Circle():
    """
    This class represents a circular obstacle

    Attributes
    ----------
    _R : float
        the radius of the circle
    _pos: np.ndarray
        the position of the circle's center 
  
    """
    def __init__(self, position: np.ndarray, radius: float) -> None:
        self._R = radius
        self._pos = position

    def display(self, ax: plt.Axes):
        """
        Displays the obstacle

        Parameters:
            ax (plt.Axes): 
                matplotlib graph
        """
        circ = plt.Circle(self._pos,self._R, color='grey', alpha=0.5, hatch='/')
        ax.add_patch(circ)

    def get_pos_vel_virtual_agent(self, drone_pose: np.ndarray, drone_vel: np.ndarray):
        """
        Creates a virtual kinematic agent on the boundary of the obstacle.
        The position of the virtual agent is the closest point on the obstacle to a drone

        Parameters:
            drone_pose (np.ndarray): 
                the position of the drone as a 1D array of shape (2,).
            drone_vel (np.ndarray): 
                the velocity of the drone as a 1D array of shape (2,).

        Returns:
            pos_virtual_agent (np.ndarray): 
                the position of the virtual agent as a 1D array of shape (2,).
            vel_virtual_agent (np.ndarray): 
                the velocity of the virtual agent as a 1D array of shape (2,).
        """
        mu = self._R / np.linalg.norm(drone_pose - self._pos)
        ak = (drone_pose - self._pos).reshape((2,1)) / np.linalg.norm(drone_pose - self._pos)
        P = np.eye(2) - ak@ak.T
        pos_virtual_agent = mu * drone_pose + (1-mu) * self._pos
        vel_virtual_agent = mu * P @ drone_vel
        return pos_virtual_agent, vel_virtual_agent

class Wall():
    """
    This class represents a rectangular obstacle

    Attributes
    ----------
    _lx : float
        the length of the wall along the x direction
    _ly : float
        the length of the wall along the y direction
    _pos: np.ndarray
        the position of the circle's center 
  
    """
    def __init__(self, position: np.ndarray, lx: float, ly: float) -> None:
        self._pos = position
        self._lx = lx
        self._ly = ly

    def display(self, ax: plt.Axes):
        """
        Displays the obstacle

        Parameters:
            ax (plt.Axes): 
                matplotlib graph
        """
        xy = self._pos - np.array([self._lx, self._ly])/2
        rec = plt.Rectangle(xy=xy, width=self._lx, height=self._ly, color='grey', alpha=0.5, hatch='/')
        ax.add_patch(rec)

    def get_pos_vel_virtual_agent(self, drone_pose: np.ndarray, drone_vel: np.ndarray) :
        """
        Creates a virtual kinematic agent on the boundary of the obstacle.
        The position of the virtual agent is the closest point on the obstacle to a drone

        Parameters:
            drone_pose (np.ndarray): 
                the position of the drone as a 1D array of shape (2,).
            drone_vel (np.ndarray): 
                the velocity of the drone as a 1D array of shape (2,).

        Returns:
            pos_virtual_agent (np.ndarray): 
                the position of the virtual agent as a 1D array of shape (2,).
            vel_virtual_agent (np.ndarray): 
                the velocity of the virtual agent as a 1D array of shape (2,).
        """
        y = np.array([
            [ self._lx, -self._ly],
            [ self._lx,  self._ly],
            [-self._lx,  self._ly],
            [-self._lx, -self._ly],
        ])/2 + self._pos

        # compute direction of the four planes:
        u = y - np.roll(y,1, axis=0)
        u = u / np.linalg.norm(u, axis=1).reshape((4,1))
        
        # compute normal vector to the four planes:
        n = np.array([u[:,1], -u[:,0]]).T

        pos_obs = np.zeros((4,2))
        vel_obs = np.zeros((4,2))
        for k in range(4):
            ak = n[k].reshape((2,1))
            P = np.eye(2) - ak@ak.T
            OP = P@drone_pose + (np.eye(2) - P)@y[k]
            vel_obs[k] = P@drone_vel
            # position of the closest point should be limited to segments of the rectangle
            OA = y[k]
            OD = np.roll(y,1, axis=0)[k]
            DA =  -OD + OA
            DP = -OD + OP
            if np.sign(np.dot(DA, DP)) < 0:
                pos_obs[k] = OD
            elif np.linalg.norm(DP) > np.linalg.norm(DA):
                pos_obs[k] = OA
            else:
                pos_obs[k] = OP
        id_closest_point = np.argmin(np.linalg.norm(pos_obs-drone_pose, axis=1))
        return pos_obs[id_closest_point], vel_obs[id_closest_point]