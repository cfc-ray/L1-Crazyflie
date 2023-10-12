# L1-Crazyflie

This project contains an implementation of the L1 Adaptive Control for Crazyflie.

## Setup

*If you're new to flying with Crazyflie, follow their official documentation to assemble the drone, download and install the client (cflib), build and flash the firmware, and configure radio permissions. You should be able to connect to the Crazyflie remotely using the Crazyradio before you move on. For more information see Bitcraze's 'getting started' page [here](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/)* 

1. **Install dependencies**

Our biggest dependencies are with the Crazyflie ecosystem. Start by installing the client and a toolchain to build the firmware with. Links can be found below
- [client](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/)
    - work up until the 'Extra' section, then return here
- [firmware](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/)
    - follow the directions to install an ARM toolchain, then return here. we'll cover the rest of the content on this page in a little bit.

From here, we add a select few dependencies of our own.
```
pip install pyyaml
```

2. **Clone this repository**
```
git clone --recursive https://github.com/cfc-ray/L1-Crazyflie.git
```

3. **Build the firmware**

The L1 adaptive controller has been implemented using the app layer in the Crazyflie firmware. This means there are no modifications needed to the main firmware - our code exists as an add-on. Follow the steps below to configure the main firmware, then compile it with the L1 controller included. You should start off in the root directory for this repo ```L1-Crazyflie```
```
cd crazyflie-firmware
make cf2_defconfig
cd ../controller_L1
make -j 12
```

4. **Flash the firmware to your Crazyflie**

You should now be in the L1 controller directory ```controller_L1```. Power on your Crazyflie and connected a Crazyradio to your computer. Then run the following command.
```
cfloader flash build/cf2.bin stm32-fw -w  <radio address>
```
where ```<radio address>``` should be replaced with the radio address of your Crazyflie. For example, if your Crazyflie is operating using radio number 01 and channel number E7, you will enter the above command as ```cfloader flash build/cf2.bin stm32-fw -w  radio://0/01/2M/E7E7E7E7E7```

At this point you should be ready to go to start flying a Crazyflie with the L1 Adaptive Control.

## Configuration

There is one configuration file: ```config.yaml```, located in the ```flight_commands/``` directory. Edit the uri in there to match the uri of your Crazyflie.

## Usage

All that's needed is to run one of the flight scripts in the ```flight_commands/``` directory. For example, to run a simple hover script that allows you to compare the L1 Adaptive Control to the stock Mellinger controller, run the following command
```
cd flight_commands
python3 hover.py
```

## License
Please read the license attached to this repository

## Further Reading
For more information about the L1 Adaptive Control check out the following publications:
- Z. Wu, S. Cheng, P. Zhao, A. Gahlawat, K. Ackerman, A. Lakshmanan, C. Yang, J. Yu, and N. Hovakimyan, " $\mathcal{L}_1$ Quad: $\mathcal{L}_1$ Adaptive Augmentation of Geometric Control for Agile Quadrotors with Performance Guarantees," *arXiv preprint arXiv:2302.07208* (2023). [(link)](https://arxiv.org/abs/2302.07208)
- N. Hovakimyan and C. Cao, $\mathcal{L}_1$ Adaptive Control Theory: Guaranteed Robustness with Fast Adaptation. *Philadelphia, PA, USA: SIAM, 2010.* [(link)](https://my.siam.org/Store/Product/viewproduct/?ProductId=1485)

For more information about the Crazyflie platform check out the official Bitcraze documentation:
- [main website](https://www.bitcraze.io/)
- [firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
