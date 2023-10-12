# L1-Crazyflie

This project contains an implementation of the L1 Adaptive Control for Crazyflie.

## Setup

*If you're new to flying with Crazyflie, follow their official documentation to assemble the drone, download and install the client (cflib), build and flash the firmware, and configure radio permissions. You should be able to connect to the Crazyflie remotely using the Crazyradio before you move on.* 

1. **Clone this repository**
```
git clone --recursive https://github.com/cfc-ray/L1-Crazyflie.git
```

2. **Build the firmware**

The L1 adaptive controller has been implemented using the app layer in the Crazyflie firmware. This means there are no modifications needed to the main firmware, we just need to compile this additional code and tell it where to insert itself in the main firmware (which we do automatically). To compile, follow the steps below
```
cd controller_L1
make
```

3. **Flash the firmware to your Crazyflie**

Ensure your Crazyflie is powered on and a Crazyradio is connected to your computer. Then run the following command.
```
cfloader flash build/cf2.bin stm32-fw -w  <radio address>
```
where ```<radio address>``` should be replaced with the radio address of your Crazyflie. For example, if your Crazyflie is operating using radio number 01 and channel E7, you will enter the above command as ```cfloader flash build/cf2.bin stm32-fw -w  radio://0/01/2M/E7E7E7E7E7```


## License
Please read the license attached to this repository

## Further Reading
For more information about the L1 Adaptive Control check out the following publications:
- Z. Wu, S. Cheng, P. Zhao, A. Gahlawat, K. Ackerman, A. Lakshmanan, C. Yang, J. Yu, and N. Hovakimyan, " $\mathcal{L}_1$ Quad: $\mathcal{L}_1$ Adaptive Augmentation of Geometric Control for Agile Quadrotors with Performance Guarantees," *arXiv preprint arXiv:2302.07208* (2023). [(link)](https://arxiv.org/abs/2302.07208)
- N. Hovakimyan and C. Cao, $\mathcal{L}_1$ Adaptive Control Theory: Guaranteed Robustness with Fast Adaptation. *Philadelphia, PA, USA: SIAM, 2010.* [(link)](https://my.siam.org/Store/Product/viewproduct/?ProductId=1485)

For more information about the Crazyflie platform check out the official Bitcraze documentation:
- [main website](https://www.bitcraze.io/)
- [firmware documentation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
