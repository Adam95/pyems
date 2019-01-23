pyems
=====

This library provides a simple interface for reading and writing to the EMS_ bus via an Arduino.

In houses where a *Buderus* boiler is installed, there probably is an EMS bus. In a standard setup there would be a boiler and a controller (for example RC35) connected together with EMS bus. One device on the bus acts as a master (bus arbitrator) and other devices as slaves.

Arduino interface
-----------------

In order for this library to be useful, you need to have your EMS bus connected to an Arduino which acts as a relay. It reads EMS on one serial line, detects complete dataframes and sends them over the other serial line (USB) to your PC running this library.

Check this link_ for information about how to build the hardware part.

Functional requirements
-----------------------

The library shall:
 - provide method for reading decoded/raw dataframes from the bus,
 - provide method for writing defined commands to the bus,
 - allow user to configure/define his own commands and dataframes.

Non-functional requirements
---------------------------

The library shall be:
 - **scalable** - allowing easy configuration of new commands for writing and dataframes for reading,
 - **robust** - reliably handle communication errors, retry sending a command if it fails, etc.


.. Links:
.. _EMS: https://domoticproject.com/ems-bus-buderus-nefit-boiler/
.. _link: TODO
