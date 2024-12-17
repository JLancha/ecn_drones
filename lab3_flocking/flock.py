import numpy as np
from typing import Dict
import matplotlib.pyplot as plt
import threading

from drone import *
from utils import compute_aij, compute_aij_obstacles, PARAMS 

class Flock():
    """
    This class represents a flock of drones

    Attributes
    ----------
    _dt : float
        The time step for updating the flock's dynamics.
    _nb_drones : int
        number of drones in the flock
    _drone : list
         A list of drones, each an object of the `Drone` class.
    _drones_pos: np.ndarray
        position of drones. Matrix of size (_nb_drones, 2)
    _drones_vel: np.ndarray
        velocity of drones. Matrix of size (_nb_drones, 2)
    _A: np.ndarray
        adjacency matrix of size (_nb_robots, _nb_robots)
    _nb_obstacles: int 
        number of obstacles in the environment
    _obstacles: list
        A list containing instances of `Sphere` or `Wall`, representing obstacles.
    _obstacles_pose: np.ndarray
        position of obstacles. Matrix of size (_nb_obstacles, 2)
    _obstacles_vel: np.ndarray
        velocity of obstacles. Matrix of size (_nb_obstacles, 2)
    _A_obs: np.ndarray
        The adjacency matrix representing interactions between drones and obstacles, of size (_nb_drones, _nb_obstacles).
    """
    
    def __init__(self, nb_agents: int, init_pose: np.ndarray, init_box_size: np.ndarray, obstacles: list, dt: float) -> None:
        """
        Constructs all the necessary attributes of a Flock

        Parameters:
            nb_agents (int): 
                the number of agent inside the flock
            init_pose (np.ndarray): 
                initial position of the center of the flock
            init_box_size (np.ndarray): 
                agents are initialized in a box of size(init_box_size[0] x init_box_size[1]) centered at init_pose 
            obstacles (list): 
                the list of obstacles
            dt (float): 
                time step
        """
        self._dt =dt

        self._nb_drones = nb_agents
        self._nb_obstacles = len(obstacles)

        self._obstacles = obstacles
        self._drone = list()
        for i in range(self._nb_drones):
            pose_agent = np.random.random(2) * init_box_size - init_box_size/2 + init_pose
            drone = Drone(pose_agent, dt)
            self._drone.append(drone)

        self._drones_pos = np.zeros((self._nb_drones,2))
        self._drones_vel = np.zeros((self._nb_drones,2))
        self._obstacles_pose = np.zeros((self._nb_drones, self._nb_obstacles,2))
        self._obstacles_vel = np.zeros((self._nb_drones, self._nb_obstacles,2))
        
        self._A = self.get_adjacency_matrix()
        self._L = self.get_laplacian_matrix()
        self._A_obs = self.get_adjacency_matrix_obstacles()
        
    def get_adjacency_matrix(self):
        """
        Returns the spacial adjacency matrix

        Returns:
            A (np.ndarray): 
                the spacial adjacency matrix of size (_nb_drones, _nb_drones)
        """
        #Ni = {j \in V: aij != 0} = {j \in V: (i,j) \in Epsilon}
        #A[i,j](q) = rho_h(||q_i - q_j||_sigma / r_alpha) \in [0,1] j != i
        # r_alpha = ||r||_sigma
        # rho_h(z) = 1 if z \in [0, h); = 0.5*[1 + cos(pi*(z-h)/(1-h))] if z \in [h, 1]; = 0 otherwise

        A = np.zeros((self._nb_drones, self._nb_drones))
        
        for i in range(self._nb_drones):
            for j in range(i+1, self._nb_drones):
                aij  = compute_aij(self._drones_pos[i], self._drones_pos[j])

                A[i,j] = aij
                A[j,i] = aij
        return A
    
    def get_laplacian_matrix(self):
        """
        Returns the spacial adjacency matrix

        Returns:
            L (np.ndarray): 
                the Laplacian of size (_nb_drones, _nb_drones)
        """
        # TODO: compute L
        L = np.zeros((self._nb_drones, self._nb_drones))
        # Get the degree matrix
        D = np.diag(np.sum(self._A, axis=1))
        #Compute the Laplacian
        L = D - self._A
        return L
    
    def get_adjacency_matrix_obstacles(self):
        """
        Returns the spacial adjacency matrix between drones and obstacles.
        If A[i,j] > 0 then _drone[i] is  is within the neighborhood of _obstacle[j]

        Returns:
            A (np.ndarray): 
                the spacial adjacency matrix of size (_nb_drones, _nb_robots)
        """
        A = np.zeros((self._nb_drones, self._nb_obstacles))
        for i in range(self._nb_drones):
            A[i,:] = compute_aij_obstacles(self._drones_pos[i], self._obstacles_pose[i]).flatten()
        return A

    def compute_connectivity(self)->float:
        """
        Returns the relative connectivity of the group

        Returns:
            rel_con (float): 
                the relative connectivity
        """
        # TODO: compute connectivity
        rank_laplacian = np.linalg.matrix_rank(self._L)
        rel_con = rank_laplacian / (self._nb_drones - 1)
        return rel_con

    def compute_deviation_energy(self)->float:
        """
        Returns the normalized deviation energy of the group
        (degree in which the flock differs form an alpha-lattice)

        Returns:
            radius (float): 
                the cohesion radius
        """
        nb_graph_edges = np.sum(self._A > 0) / 2

        # TODO: compute deviation_energy
        #E_normalized = E(q)/d^2
        #E(q) = 1/(1+eps(q)) * sum_{i,j} A_{ij} (||q_i - q_j|| - d)^2
        self._A = self.get_adjacency_matrix()
        energy = 1/(1+nb_graph_edges) * np.sum(np.sum(self._A * (np.linalg.norm(self._drones_pos - self._drones_pos[:,None], axis=2) - PARAMS["d"])**2))
        energy = energy / PARAMS["d"]**2
        return  energy
    
    def compute_velocity_mismatch(self)->float:
        """
        Returns the normalized velocity mismatch of the group
        (kinetic energy of the group)

        Returns:
            vel (float): 
                the velocity mismatch
        """
        # TODO: compute velocity mismatch
        
        #Average (center) velocity:
        vel = np.mean(self._drones_vel, axis=0)
        #Compute the velocity mismatch
        vel = np.linalg.norm(self._drones_vel - vel)**2
        #Normalize the velocity mismatch
        vel = 0.5*vel / self._nb_drones
        
        return vel

    def compute_cohesion_radius(self)->float:
        """
        Returns the cohesion radius of the group (maximal distance between drones)

        Returns:
            radius (float): 
                the cohesion radius
        """
        # TODO: compute cohesion 
        #Compute the position of the centroid of the flock qc:
        qc = np.mean(self._drones_pos, axis=0)
        #Compute the distance between the centroid and each drone
        radius = np.max(np.linalg.norm(self._drones_pos - qc, axis=1))
        return radius

    def update_flock(self, goal_pos: np.ndarray, goal_vel: np.ndarray):  
        """
        Updates the states, graph and control actions of the flock

        Parameters:
            goal_pos (np.ndarray): 
                target position of the center of mass of the flock
            goal_vel (np.ndarray): 
                target velocity of the center of mass of the flock
        """
        # update the state of the flock
        for i in range(self._nb_drones):
            self._drones_pos[i,:] = self._drone[i].get_position()
            self._drones_vel[i,:] = self._drone[i].get_velocity()
            for j in range(self._nb_obstacles):
                self._obstacles_pose[i,j,:], self._obstacles_vel[i,j,:] = self._obstacles[j].get_pos_vel_virtual_agent(self._drones_pos[i], self._drones_vel[i])
        
        # update the graphs
        self._A = self.get_adjacency_matrix()
        self._L = self.get_laplacian_matrix()
        self._A_obs = self.get_adjacency_matrix_obstacles()

        # update the command input of the drones
        for i in range(self._nb_drones):
            neighbors_drones_pos = self._drones_pos[self._A[i]>0]
            neighbors_drones_vel = self._drones_vel[self._A[i]>0]
            neighbors_obst_pos = self._obstacles_pose[i, self._A_obs[i]>0]
            neighbors_obstacles_vel = self._obstacles_vel[i, self._A_obs[i]>0]

            self._drone[i].update_cmd(neighbors_drones_pos, neighbors_drones_vel, neighbors_obst_pos,neighbors_obstacles_vel, goal_pos, goal_vel)

    def display_flock(self, ax: plt.Axes):
        """
        Displays the flock

        Parameters:
            ax (plt.Axes): 
                matplotlib graph
        """
        for i in range(self._nb_drones):
            self._drone[i].diplay_agent(ax)
            for j in range(self._nb_obstacles):
                if self._A_obs[i,j]>0:
                    Xi, Xj = self._drones_pos[i], self._obstacles_pose[i,j]
                    ax.plot([Xi[0], Xj[0]], [Xi[1], Xj[1]], color="red")

        for i in range(self._nb_drones):
            for j in range(i+1, self._nb_drones):
                if self._A[i,j] > 0:
                    Xi, Xj = self._drone[i].get_position(), self._drone[j].get_position()
                    ax.plot([Xi[0], Xj[0]], [Xi[1], Xj[1]], alpha=self._A[i,j], color="blue")
        mean = np.mean(self._drones_pos, axis=0)
        ax.scatter(mean[0], mean[1], color="purple", marker="x")
        ax.set_xlabel("x(m)")
        ax.set_ylabel("y(m)")