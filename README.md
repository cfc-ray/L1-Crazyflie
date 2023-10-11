# L1-Crazyflie

This project contains an implementation of the L1 Adaptive Control for Crazyflie.

## Setup
1. clone the repository
```git clone --recursive https://github.com/cfc-ray-L1-Crazyflie```

2. configure the firmware
```cd crazyflie-firmware```
```make cf2_defconfig```
```cd ..```

3. build the firmware
```make -j 8```

4. flash the firmware to your Crazyflie
```cfloader flash build/cf2.bin stm32-fw -w  <radio address>```
    where ```<radio address>``` should be replaced with the radio address of your Crazyflie. For example, if your Crazyflie is operating using radio number 01 and channel E7, you will enter the above command as ```cfloader flash build/cf2.bin stm32-fw -w  radio://0/01/2M/E7E7E7E7E7```