'''
This script commands the Crazyflie to fly in a square pattern using the L1 Adaptive Control.

Parameters
    HEIGHT: the height the drone should fly at, in meters
    LENGTH: the side length of the square, in meters

'''

import logging
import time
import cflib.crtp

from simple_client import SimpleClient


HEIGHT = 0.4
LENGTH = 1


def main():

    # setup 
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient()
    while not client.is_connected:
        print(f' ... connecting ...')
        time.sleep(1.0)

    # Leave time at the start to initialize
    client.stop(1.0)

    # reset state estimate and set controller to Mellinger
    client.cf.param.set_value('kalman.resetEstimation', 1)
    client.cf.param.set_value('stabilizer.controller', 2)

    # pause to finish initialization
    time.sleep(0.1)

    # ------------------------------------------------------------------------------
    # FLIGHT COMMANDS START HERE: 

    # takeoff
    print('\ntakeoff...\n')
    client.takeoff(HEIGHT, dt=3)

    # activate the L1 controller
    client.cf.param.set_value('stabilizer.controller', 5)

    # fly in a square, pausing at each corner
    client.move_smooth([0., 0., HEIGHT], [0., LENGTH, HEIGHT], 0., 4.0)
    client.move(0, LENGTH, HEIGHT, 0, 1)

    client.move_smooth([0., LENGTH, HEIGHT], [LENGTH, LENGTH, HEIGHT], 0., 4.0)
    client.move(LENGTH, LENGTH, HEIGHT, 0, 1)

    client.move_smooth([LENGTH, LENGTH, HEIGHT], [LENGTH, 0., HEIGHT], 0., 4.0)
    client.move(LENGTH, 0, HEIGHT, 0, 1)

    client.move_smooth([LENGTH, 0., HEIGHT], [0., 0., HEIGHT], 0., 4.0)
    client.move(0, 0, HEIGHT, 0, 1)

    # land
    print('\nlanding...\n')
    client.cf.param.set_value('stabilizer.controller', 2)
    client.move_smooth([0., 0., HEIGHT], [0., 0., 0.1], 0., 1.0)
    client.land()

    # ------------------------------------------------------------------------------

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    client.write_data('square_data.json')


if __name__ == '__main__':
    main()