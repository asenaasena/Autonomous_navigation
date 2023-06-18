#!/usr/bin/env python
import glob
import os
import sys
import random
import time
import numpy as np
import cv2

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list = []
IM_WIDTH = 640
IM_HEIGHT = 480

try:
    client = carla.Client("localhost", 2000)
    client.set_timeout(3.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    veh_bp = blueprint_library.filter("model3")[0]
    veh_bp.set_attribute('role_name', 'ego_vehicle')
    #carla.synchronous_mode(True)

    #spawn_point = random.choice(world.get_map().get_spawn_points())
    spawn_point = carla.Transform(carla.Location(x=-7.46, y=202.164, z=2.00), carla.Rotation(pitch=0.0, yaw =90.0, roll=0.0))
    vehicle = world.spawn_actor(veh_bp, spawn_point)
    control_vehicle = carla.VehicleControl()
    vehicle.apply_control(control_vehicle)
    actor_list.append(vehicle)
    spectator = world.get_spectator()
    #while True:
     #       spectator_transform =  vehicle.get_transform()
     #       spectator_transform.location += carla.Location(x = 0, y=-7, z = 3.0)
     #       spectator.set_transform(spectator_transform)
      #      time.sleep(0.5)


    spectator_transform = vehicle.get_transform()
    spectator_transform.location += carla.Location(x = 0, y=-10, z = 2.0)
    spectator.set_transform(spectator_transform)
    #vehicle.set_autopilot(True)
    #vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer = 0.0))
    actor_list.append(vehicle)

    for i in range(10):
        time.sleep(2)
        #print(vehicle.get_location())
        #print(vehicle.get_rotation())
        print(vehicle.get_transform())
        
    #CAmera sensor set up
    # cam_bp = blueprint_library.find('sensor.camera.rgb')
    # cam_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    # cam_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
    # cam_bp.set_attribute("fov", "110")
    # spawn_point = carla.Transform(carla.Location(x =2.5, z=0.7))
    # #camera_location = carla.Location(0,0,0)
    # #camera_rotation = carla.Rotation(0,180,0)
    # sensor = world.spawn_actor(cam_bp, spawn_point, attach_to = vehicle)
    
    # actor_list.append(sensor)

    # sensor.listen(lambda data: process_img(data))
    

    time.sleep(100.0)

finally:
    for actor in actor_list:
        actor.destroy()
    print("all clean")