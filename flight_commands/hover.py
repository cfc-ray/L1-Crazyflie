'''
This script commands the Crazyflie to hover and allows the user to activate and deactivate the L1 Adaptive Control, to showcase its performance.
It uses threading to simultaenously send flight commands to the drone and handle user inputs.

Parameters
    HEIGHT: the height the drone should fly at, in meters

Usage
    enter "python3 hover.py XX" in a terminal, replacing XX with the radio number for your Crazyflie
'''

import logging
import time
import threading
import cflib.crtp
import argparse

from simple_client import SimpleClient


HEIGHT = 0.4


def main(radio: int):

    # configure uri
    uri = 'radio://0/' + str(radio) + '/2M/E7E7E7E7E7'

    # setup 
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient(uri)
    while not client.is_connected:
        print(f' ... connecting ...')
        time.sleep(1.0)

    # Leave time at the start to initialize
    client.stop(1.0)

    # reset state estimate and set controller to Mellinger
    client.cf.param.set_value('kalman.resetEstimation', 1)
    client.cf.param.set_value('stabilizer.controller', 2)

    # setup threads
    flight_thread = threading.Thread(target=client.hover, args=[0., 0., HEIGHT, 0.])

    # pause to finish initialization
    time.sleep(0.1)

    # ------------------------------------------------------------------------------
    # FLIGHT COMMANDS START HERE: 

    # takeoff
    print('\ntakeoff...\n')
    client.takeoff(HEIGHT, dt=3)

    # hover
    flight_thread.start()
    while client.remain_hovering:
        print(f"L1 is currently {'ON' if client.cf.param.get_value('stabilizer.controller') == 5 else 'OFF'}")
        val = input("Enter '1' for L1 on, enter '0' for L1 off, enter nothing to land: ")
        if val:
            try:
                val = int(val)
                if val == 1:
                    client.cf.param.set_value('stabilizer.controller', 5)
                elif val == 0:
                    client.cf.param.set_value('stabilizer.controller', 2)
                else:
                    print(f"value {val} is invalid! please enter a '1', '0', or nothing.")
            except:
                print(f"failed to set controller!\nending demo...")
                client.remain_hovering = False
                client.stop(0.5)
                client.disconnect()
        else:
            client.remain_hovering = False
        
        time.sleep(0.1)

    # land
    print('\nlanding...\n')
    client.cf.param.set_value('stabilizer.controller', 2)
    client.move_smooth([0., 0., HEIGHT], [0., 0., 0.1], 0., 1.0)
    client.land()

    # ------------------------------------------------------------------------------

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    client.write_data('hardware_data.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("radio")
    args = parser.parse_args()
    try:
        radio = int(args.radio)
    except:
        print("make sure to include drone radio number (as integer)! exiting...")
        import sys; sys.exit()

    main(radio)