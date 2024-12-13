from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gz_ressource_path = get_package_share_directory("lab_auve") + "/gazebo"
    gz_env = {"IGN_GAZEBO_RESOURCE_PATH" : gz_ressource_path+"/models"}
    gz_start = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', gz_ressource_path+'/worlds/single_uav.sdf'],
        additional_env=gz_env
        )
    return LaunchDescription([gz_start])
