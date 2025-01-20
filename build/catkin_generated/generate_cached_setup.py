# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/ant/work/autoware.ai/install/waypoint_maker;/home/ant/work/autoware.ai/install/voice_status;/home/ant/work/autoware.ai/install/vehicle_websocket;/home/ant/work/autoware.ai/install/vehicle_tcp;/home/ant/work/autoware.ai/install/vehicle_odom;/home/ant/work/autoware.ai/install/vehicle_description;/home/ant/work/autoware.ai/install/op_local_planner;/home/ant/work/autoware.ai/install/op_global_planner;/home/ant/work/autoware.ai/install/lidar_kf_contour_scan;/home/ant/work/autoware.ai/install/op_ros_helpers;/home/ant/work/autoware.ai/install/op_simu;/home/ant/work/autoware.ai/install/op_planner;/home/ant/work/autoware.ai/install/op_utility;/home/ant/work/autoware.ai/install/vector_map_server;/home/ant/work/autoware.ai/install/map_file;/home/ant/work/autoware.ai/install/lane_planner;/home/ant/work/autoware.ai/install/vector_map;/home/ant/work/autoware.ai/install/vector_map_msgs;/home/ant/work/autoware.ai/install/vector_app;/home/ant/work/autoware.ai/install/uwb_togps;/home/ant/work/autoware.ai/install/twist_gate;/home/ant/work/autoware.ai/install/twist_filter;/home/ant/work/autoware.ai/install/tablet_socket_msgs;/home/ant/work/autoware.ai/install/startingup_ros;/home/ant/work/autoware.ai/install/socketcan_bridge;/home/ant/work/autoware.ai/install/socketcan_interface;/home/ant/work/autoware.ai/install/sensor_connect;/home/ant/work/autoware.ai/install/sanchi_amov;/home/ant/work/autoware.ai/install/rslidar_sdk;/home/ant/work/autoware.ai/install/rplidar_ros;/home/ant/work/autoware.ai/install/pure_pursuit;/home/ant/work/autoware.ai/install/lidar_localizer;/home/ant/work/autoware.ai/install/autoware_health_checker;/home/ant/work/autoware.ai/install/ros_observer;/home/ant/work/autoware.ai/install/points_downsampler;/home/ant/work/autoware.ai/install/pointcloud_to_laserscan;/home/ant/work/autoware.ai/install/pcl_omp_registration;/home/ant/work/autoware.ai/install/ntrip_ros;/home/ant/work/autoware.ai/install/nmea_navsat_driver;/home/ant/work/autoware.ai/install/nlink_parser;/home/ant/work/autoware.ai/install/ndt_cpu;/home/ant/work/autoware.ai/install/multi_laser;/home/ant/work/autoware.ai/install/lslidar_driver;/home/ant/work/autoware.ai/install/lslidar_msgs;/home/ant/work/autoware.ai/install/lslidar_c16_decoder;/home/ant/work/autoware.ai/install/lslidar_c16_driver;/home/ant/work/autoware.ai/install/lslidar_c16_msgs;/home/ant/work/autoware.ai/install/livox_ros_driver2;/home/ant/work/autoware.ai/install/lio_sam_6axis;/home/ant/work/autoware.ai/install/libwaypoint_follower;/home/ant/work/autoware.ai/install/lanelet2_extension;/home/ant/work/autoware.ai/install/interface_udp;/home/ant/work/autoware.ai/install/gps_imu;/home/ant/work/autoware.ai/install/gnss_localizer;/home/ant/work/autoware.ai/install/gnss;/home/ant/work/autoware.ai/install/dgps_ros;/home/ant/work/autoware.ai/install/detected_objects_visualizer;/home/ant/work/autoware.ai/install/cv_bridge_1;/home/ant/work/autoware.ai/install/can_connect;/home/ant/work/autoware.ai/install/can_client;/home/ant/work/autoware.ai/install/cam_pose_converter;/home/ant/work/autoware.ai/install/autoware_system_msgs;/home/ant/work/autoware.ai/install/autoware_remove_msgs;/home/ant/work/autoware.ai/install/autoware_connector;/home/ant/work/autoware.ai/install/amathutils_lib;/home/ant/work/autoware.ai/install/autoware_msgs;/home/ant/work/autoware.ai/install/autoware_lanelet2_msgs;/home/ant/work/autoware.ai/install/autoware_config_msgs;/home/ant/work/autoware.ai/install/autoware_can_msgs;/home/ant/work/autoware.ai/install/autoware_build_flags;/home/ant/work/autoware.ai/install/area_planning;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ant/work_test/src/lidar_lane_detection/build/devel/env.sh')

output_filename = '/home/ant/work_test/src/lidar_lane_detection/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
