This repo contains a ROS device driver for the BOSCH IndraDrive CS, an industrial motor controller made by Bosch Rexroth.
This package uses the odva_ethernet library (available [here](https://github.com/ros-drivers/odva_ethernetip)) to communicate with the controller over EtherNet/IP.

# Configuring the drive
The following configuration was saved to the controller. This package assumes you are also using these settings. You may need to ajust the relevant settings in this package if your drive configuration is different.

1. Connect to the controller using IndraWorks DS
<p align="center">
  <img src="https://raw.githubusercontent.com/markjones112358/bosch_indra_drive_cs/master/doc/1.PNG" alt="IndraWorks DS Connection Page"/>
</p>
2. Ensure EtherNet/IP has been enabled on the drive as the active communication protocol.
<p align="center">
  <img src="https://raw.githubusercontent.com/markjones112358/bosch_indra_drive_cs/master/doc/2.PNG" alt="EtherNet/IP Setting"/>
</p>
3. Configure the IP address, Configuration connection point, assembly connection points (both AT and MDT).
<p align="center">
  <img src="https://raw.githubusercontent.com/markjones112358/bosch_indra_drive_cs/master/doc/3.PNG" alt="EtherNet/IP Configuration"/>
</p>
4. Load the default IO messages into the drive. If your settings differ here, you will need to adjust include/bosch_indra_driver/measurement_report.h and include/bosch_indra_driver/measurement_report_config.h to match.
<p align="center">
  <img src="https://raw.githubusercontent.com/markjones112358/bosch_indra_drive_cs/master/doc/5.PNG" alt="Configure AT and MDT messages"/>
</p>
<p align="center">
  <img src="https://raw.githubusercontent.com/markjones112358/bosch_indra_drive_cs/master/doc/6.PNG" alt="Configure AT and MDT messages"/>
</p>
5. Ensure the appropriate drive configuration has been set for this application.
<p align="center">
  <img src="https://raw.githubusercontent.com/markjones112358/bosch_indra_drive_cs/master/doc/7.PNG" alt="Configure AT and MDT messages"/>
</p>
6. Commission the motor for use with the drive if you haven't already. Ensure that you can jog the motor manually using IndraWorks DS before trying to control the motor using this package. Doing so will eliminate failure due to drive misconfiguration.
