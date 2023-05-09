# robo_hand_v2.0
project build on stm32f401ccu microcontroller and NRF24L01. 

Robot manipulator was printed on 3d printer Creality ender-3 v2, plastic PETg. 
Remoute has 5 potentiometers, nrf24L01+ module and 18650 battery. Remoute transmiting 10 Bytes to RX module (robot manipulator). Acordint this received data microcontroller on manipulator generate neeaded PWM signals for all 5 servo morors MG995. Also for make right voltage of signal fom stm32 (3.3v) to MG995 (5v) was used (Translation - Voltage Levels SingleB Dual-Supply Bus Trans SN74LVC1T45DBVR).The received data sent over UART to PC.

![alt text](https://github.com/OlegDemk/robo_hand_v2.0/blob/main/photo_1.jpg)

![alt text](https://github.com/OlegDemk/robo_hand_v2.0/blob/main/photo_2.jpg)

![alt text](https://github.com/OlegDemk/robo_hand_v2.0/blob/main/remote.jpg)

![alt text](https://github.com/OlegDemk/robo_hand_v2.0/blob/main/data.png)
