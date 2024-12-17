import numpy as np
import matplotlib.pyplot as plt

from flock import Flock
from obstacles import Circle, Wall

# Flocking parameters
NB_AGENTS = 12 # numbe of agents in the flock
Nt = 600 # number of simulation steps
OBSTACLES = [
    Circle(np.array([15,2]), 2.),
    Circle(np.array([30,7]), 6.),
    Wall(np.array([45,15]), 6,20),
    Wall(np.array([45,-15]), 6,20),
]
INIT_POSE_FLOCK = np.array([-10,0]) # initial position of the flock
FINAL_POSE_FLOCK = np.array([70,0])
INIT_AREA_FLOCK = np.array([20,20]) # agent are initialized inside a box of size (INIT_AREA_FLOCK[0] x INIT_AREA_FLOCK[1])

DT = 0.05

def trajectory(init_pose: np.ndarray, final_pose: np.ndarray):
    """
     Return a trajectory using a polynome interpolation

        Parameters:
            init_pose (np.ndarray): the start position
            final_pose (np.ndarray): the final position

        Returns:
            pos (np.ndarray): the position trajectory size (Nt, 2)
            vel (np.ndarray): the velocity trajectory size (Nt, 2)
    """
    tf = Nt*DT
    t = np.linspace(0, tf, Nt).reshape((Nt,1))
    s = (-1/(2*tf**3) * t**3 + 3/(2*tf**2) * t**2)
    ds = (-3/(2*tf**3) * t**2 + 6/(2*tf**2) * t)
    pos = init_pose + s * final_pose
    vel = ds * final_pose
    return pos, vel

def main():
    dt=DT
    # create the flock
    flock = Flock(
        nb_agents=NB_AGENTS, 
        init_pose=INIT_POSE_FLOCK,
        init_box_size=INIT_AREA_FLOCK,
        obstacles=OBSTACLES,
        dt=dt
    )
    
    fig1, ax1 = plt.subplots()
    fig2, ax2 = plt.subplots(2,2)
    ax2 = ax2.flatten()
    connectivity = []
    energy_deviation = []
    vel_mismatch = []
    cohesion_rad = []
    tps = []

    traj_pos, traj_vel = trajectory(INIT_POSE_FLOCK, FINAL_POSE_FLOCK)

    i=-1
    while(True):
        i+=1

        # update the flock
        if i < Nt:
            flock.update_flock(traj_pos[i], traj_vel[i])
        else:
            flock.update_flock(traj_pos[-1], np.zeros(2))
        
        # plot the flock
        ax1.cla()
        flock.display_flock(ax1)
        ax1.scatter(traj_pos[min(i, Nt-1),0], traj_pos[min(i, Nt-1),1], marker=".", color = "green")
        ax1.plot(traj_pos[:min(i, Nt-1),0], traj_pos[:min(i, Nt-1),1], linestyle="--", color="green", alpha=0.5)
        for obstacle in OBSTACLES:
            obstacle.display(ax1)
        fig1.suptitle(f"t = {round(i*dt, 4)} s")
        ax1.set_xlim(INIT_POSE_FLOCK[0]-INIT_AREA_FLOCK[0]/2, FINAL_POSE_FLOCK[0]+20)
        ax1.set_ylim(-25,25)
        ax1.set_aspect('equal')
        fig1.canvas.draw() 

        # plot connectivity
        tps.append(i*DT)
        connectivity.append(flock.compute_connectivity())
        ax2[0].plot(tps, connectivity, color="blue")
        ax2[0].set_xlabel("t(s)")
        ax2[0].set_ylabel(r"$\lambda_2$")
        ax2[0].set_title(f"Connectivity")
        # # plot deviation energy
        energy_deviation.append(flock.compute_deviation_energy())
        ax2[1].plot(tps, energy_deviation, "green")
        ax2[1].set_xlabel("t(s)")
        ax2[1].set_ylabel("deviation energy")
        ax2[1].set_title(f"Deviation Energy")
        # # plot velocity mismatch
        vel_mismatch.append(flock.compute_velocity_mismatch())
        ax2[2].plot(tps, vel_mismatch, color="orange")
        ax2[2].set_xlabel("t(s)")
        ax2[2].set_ylabel("velocity mismatch")
        ax2[2].set_title(f"Velocity Mismatch")
        # # plot cohesion radius
        cohesion_rad.append(flock.compute_cohesion_radius())
        ax2[3].plot(tps, cohesion_rad, color="purple")
        ax2[3].set_xlabel("t(s)")
        ax2[3].set_ylabel("radius(m)")
        ax2[3].set_title(f"Cohesion Radius")
        fig2.canvas.draw()
        
        plt.pause(0.0001)

if __name__=="__main__":
    main()
    plt.show()