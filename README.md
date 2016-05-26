# MikroKopterFlightController
Special Version of the MikroKopter Firmware for simplified and improved control with external systems

Two branches: 

One based on Version V2.00a and one based on Version V2.12a. Version V2.12a is also compatiple with more recent hardware revisions.

Version V2.00a was choosen because (at this time) most updated version V2.08a was not working properly on our system.
Original Source: http://svn.mikrokopter.de/websvn/listing.php?repname=FlightCtrl&path=%2Ftags%2FV2.00a_NOT_for_FC_V2.5%2F&#a4f0e706c6bd462259eead4c87f23de7a

Original Source of V2.12a: http://svn.mikrokopter.de/websvn/listing.php?repname=FlightCtrl&path=%2Ftags%2FV2.12a%2F&#acd55e8333066e5326e659aa13487f8e9

This special version includes adjustments for
* Proper compilation in Linux Environment (tested on Ubuntu 14.04)
* Disabled time limited subscription of debug stream (Can be enabled and disabled)
* Disabled minimum thrust/throttle selection on external control instead thrust/throttle are set directly
* Integrated new UART commands:
  * remote arming('x')/disarming('e')
  * triggering the calibration ('r' with payload 1 for preflight calibration and 2 for persisted calibration to EEPROM)
  * beep sound ('o' with beep length (int16) as payload)
