# MikroKopterFlightController
Special Version of the MikroKopter Firmware for simplified and improved control with external systems

Based on Version V2.00a because (at this time) most updated version V2.08a was not working properly on our system.
Original Source: http://svn.mikrokopter.de/websvn/listing.php?repname=FlightCtrl&path=%2Ftags%2FV2.00a_NOT_for_FC_V2.5%2F&#a4f0e706c6bd462259eead4c87f23de7a

This special version includes adjustments for
* Proper compilation in Linux Environment (tested on Ubuntu 14.04)
* Disabled time limited subscription of debug stream (Can be enabled and disabled)
* Disabled minimum thrust/throttle selection on external control instead thrust/throttle are set directly
