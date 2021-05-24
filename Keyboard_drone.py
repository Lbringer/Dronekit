from dronekit import VehicleMode,connect
from pymavlink import mavutil
import time
import pygame

pygame.init()
pygame.display.set_mode((150,150))

gnd_speed = 5

print('Connecting to vehicle')

vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("[INFO] waiting to fly....")
        time.sleep(1)
    print("[INFO] arming motors")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("[INFO] waiting to arm")
        time.sleep(1)

    print("[INFO] Taking off....")
    vehicle.simple_takeoff(altitude)

    while True:
        print('[INFO] Altitude{}'.format(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= 0.95 * altitude:
            print("[INFO] Target altitude reached")
            break
        time.sleep(1)


arm_and_takeoff(75)

def send_ned_velocity(x,y,z):
    msg=vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,0,0,
        x,y,z,
        0,0,0,
        0,0
    )
    for i in range(0,5):
      vehicle.send_mavlink(msg)
      time.sleep(1)
    return True

if __name__ == "__main__":
    print("Ready to control")
    while 1:
       for event in pygame.event.get():
           if event.type == pygame.KEYDOWN:
               if event.key == pygame.K_UP:
                send_ned_velocity(gnd_speed,0,0)
               elif event.key == pygame.K_DOWN:
                  send_ned_velocity(-gnd_speed,0,0)
               elif event.key == pygame.K_LEFT:
                  send_ned_velocity(0,-gnd_speed,0)
               elif event.key == pygame.K_RIGHT:
                  send_ned_velocity(0,gnd_speed,0)         
