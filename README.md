# LPSensor_App
This project holds LPC845 firmware code for Low Power Sensor project.
  - About Low Power Sensor App
  - Connection Diagram
  - Prerequisities
    - Installing MCUXpresso IDE
    - Installing SDK2.10 LPC845
    - Build and Load
    - Debugging
  - Utilized resources

# About Low Power Sensor App 
In this project, NXP LPC485 MCU ve Bosch BME280 sensor have been used.
BME280 sensor can monitor the value of the temperature, humidity and pressure values of the ambience.
Project has been designed for the possible maximum battery life.

For the high power consumption:
  - LPC845 processor has been used in 'Power Down' mode.
  - Also MCU has been driven to get up by using the "Self-Wake-up Timer" which has been designed
      specifically for low-power systems.
   - The power consumption is 0.4uA when Deep Power Down ve WKT are together used.[1]
   -  BME280 is configured for low Weather Monitoring. The total power consumption of the used structure
      is equal to 0.16uA. However, in precision and noise subjects, it is less successful.[2]   
![](docs/FlowCart.jpg)

# Connection Diagram
![](docs/Connection_Diagram.jpg)
# Prerequisities
In order to develop firmware for LPSensorApp , you will need:
  - MCUXpresso IDE
  - A UART to USB converter is required for the connection between PC and Board.
  - Serial Terminal for user interface
# Installing MCUXpresso IDE
  - [MCUXpresso IDE Download](https://www.nxp.com/webapp/swlicensing/sso/downloadSoftware.sp?catid=MCUXPRESSO)
# Installing SDK2.10 LPC845
  - [MCUXpresso SDK Download](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-software-development-kit-sdk:MCUXpresso-SDK)
# Build and Load
  - Build and Load with Linkserver, Jlink or ISP
# Debugging
  -  LCP845 IO_25 ---> Tx
  -  LPC845 IO_24 ---> Rx
  -  Connect pins to usb to serial converter.Then connect converter to pc via USB.
  -  Open serial terminal and select serial port.
  -  Serial Port Config:
        - Baud Rate : 115200
        - Parity    : None
        - Data Bits : 8
        - Stop Bits : 1    
 # Utilized resources
   - [LPC84x Datasheet](https://www.nxp.com/docs/en/data-sheet/LPC84x.pdf)
   - [LPC84x UserManual](https://cache.nxp.com/secured/assets/documents/en/user-guide/UM11181.pdf?fileExt=.pdf&__gda__=1653048191_38f441dd1ac58c2d39502e3df2d9ab67)
   - [BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
   - [BME280 Library](https://github.com/BoschSensortec/BME280_driver)
 # References
   -  [1] LPC845 DataSheet Table-13
   -  [2] BME280 DataSheet Section 3.5.1 Weather Monitoring
