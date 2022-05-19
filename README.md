# ArduPlane Modification for Small UAS System Identification
This README details the modifications made to the ArduPlane source code to support system identification flight tests for small fixed-wing UAS. Modifications are found in the "sysid_mod" branch. The original ArduPilot README can be found after this section. The modifications made to this code were designed to be configurable during flight tests. All the modified code will be detailed in this README. This code was used to carry out flight tests and collect data used in the paper "Frequency domain system identification of a small flying-wing UAS" by Justin Matt et al., presented at AIAA SCITECH 2022. 

## Overview
The original modifications implement the ability to perform automated frequency sweep maneuvers by generating signals and sending them to the aileron, elevator, or rudder channels. This was implemented by modifying the TRAINING flight mode. ArduPilot parameters were added so that the system identification code can be activated and configured from a GCS.

Second, the ability to add a frequency sweep disturbance to the actuator channels was implemented in FBWA mode. This allows for identification of the loop broken at the actuators, which is used to compute stability margins of the system. Similarly, these options are configurable using ArduPilot parameters.

This fork modified ArduPlane version 4.0.9, which was released on February 23rd, 2021. The ArduPilot code was frozen since that point. It was NOT rebased. All modifications were solely to the system ID modification.

There are many files that were modified. The majority of files are located in the ArduPlane directory. To implement the broken loop code, the ArduPlane roll and pitch controllers, located in the libraries directory, were also modified. Each modified file is listed here, with a brief description of the modifications. All modifications are commented in the main files to show the date and author of the modification. 

- **[ArduPlane/ArduPlane.cpp](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/ArduPlane.cpp)**
  - This is the main loop of the ArduPlane code. A couple of lines were added to update_control_mode function for initialization of the frequency sweep code.
- **[ArduPlane/Attitude.cpp](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/Attitude.cpp)**
  - This is the main code for high-level control. The stabilize_roll and stabilize_pitch functions, which call the roll and pitch controllers, were modified to use a custom PD controller. Additionally, if statements were added to determine if the off-axis controller should be active during frequency sweeps depending on parameters.
- **[ArduPlane/Log.cpp](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/Log.cpp)**
  - A custom log message was added to this file to record the signals of the frequency sweep with higher priority. At the time of initial modification, the AETR variable was not saved by ArduPilot, and so a custom message was needed. This is likely not needed anymore. 
- **[ArduPlane/Parameters.cpp](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/Parameters.cpp)** and **[ArduPlane/Parameters.h](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/Parameters.h)**
  - Custom parameters are defined here as part of the G2 class of parameters. These modifications follow the develop guide found [here](https://ardupilot.org/dev/docs/code-overview-adding-a-new-parameter.html). All custom parameters start with "j." The next section will detail all the parameters.
- **[ArduPlane/Plane.h](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/Plane.h)**
  - This is the header file for the main Plane class, which is used throughout the code. New variables are defined here. 
- **[ArduPlane/mode_fbwa.cpp](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/mode_fbwa.cpp)**
  - This is the Fly-by-Wire A (FBWA) code. This was modified to include broken loop sweeps if certain parameters are defined by the user. The modifications generate an automated frequency sweep, saved in the u_sweep variable. A logger is included to record the signals specific to the broken loop sweeps.
- **[ArduPlane/mode_training.cpp](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/mode_training.cpp)**
  - This is the TRAINING mode code. This was modified to generate open-loop frequency sweeps if certain parameters are defined. Similarly, the sweep is saved in the u_sweep variable. Automated doublets were also programmed in. Additionally, two safety measures were implemented. A type of "bang-bang" style control was implemented to add small, square inputs if the aircraft's roll or pitch angle exceeds a certain limit. Then, if the roll or pitch exceeds a (larger) limit, the frequency sweep aborts and the standard roll and pitch controllers take over. These were flight tested once and work as implemented but were not very effective. These algorithms can effectively be disabled by setting the parameters they rely on to certain values.
- **[ArduPlane/servos.cpp](https://github.com/justinjmatt/ardupilot/blob/master/ArduPlane/servos.cpp)**
  - This file sends the signals generated by the radio, controllers, etc. to the servos. This was modified to add the frequency sweep signal, stored in u_sweep, to the respective channel. The custom log message is called from this file.
- **libraries/APM_Control/[AP_RollController.cpp](https://github.com/justinjmatt/ardupilot/blob/master/libraries/APM_Control/AP_RollController.cpp), [AP_RollController.h](https://github.com/justinjmatt/ardupilot/blob/master/libraries/APM_Control/AP_RollController.h), [AP_PitchController.cpp](https://github.com/justinjmatt/ardupilot/blob/master/libraries/APM_Control/AP_PitchController.cpp),** and **[AP_PitchController.h](https://github.com/justinjmatt/ardupilot/blob/master/libraries/APM_Control/AP_PitchController.h)**
  - These files were modified to implement new PD controllers which are simple compared to the ArduPilot controllers. Variables defined in the header files.

## jParameter Definitions
Many parameters were defined to allow for the system identification flights to be configurable in the field. They each start with "j", and can be configured from a GCS such as Mission Planner. Information can also be found in ArduPlane/Parameters.cpp.

**jSYSID_AXIS**

Defines the axis that the automated signals are performed. 
- 1: roll w/ pitch controller inactive
- 2: pitch w/ roll controller inactive
- 3: yaw w/ pitch controller inactive
- 4: roll w/ pitch controller active
- 5: pitch w/ roll controller active
- 6: yaw w/ pitch controller active

**jSYSID_TYPE**

Defines the type of maneuver.
- 0: manual sweeps (same as manual mode but with saturation on the control magnitude)
- 1: automated sweeps in TRAINING mode
- 2: broken loop (at actuator) sweeps in FBWA mode
- 3: automated doublets in TRAINING mode

**jSYSID_AMP**

Amplitude of the automated maneuver, defined as a percentage of the maximum command signal. Range is [0, 100].

**jSYSID_T_REC**

Length of the automated frequency sweep in seconds. 

**jSYSID_F_MIN_HZ**

Minimum frequency of frequency sweep in Hz. There will be one period of constant frequency at this value before progressing through the sweep.

**jSYSID_F_MAX_HZ**

Maximum frequency of frequency sweep in Hz.

**jSYSID_T_FADEIN**

Length of fade-in to maximum magnitude for frequency sweeps. The magnitude increases linearly until reaching the value set by jSYSID_AMP at this time.

**jSYSID_Theta0**

Trim pitch angle to hold during sweeps where the pitch controller is active (e.g. roll sweeps with jSYSID_AXIS = 4)

**jPITCH_KP**

P gain for the custom pitch controller. Defined in units of radians for pitch measurements and elevator deflection, so that it is compatible with system ID models.

**jPITCH_KD**

D gain for the custom pitch controller. Defined in units of radians for pitch measurements and elevator deflection, so that it is compatible with system ID models.

**jROLL_KP**

P gain for the custom roll controller. Defined in units of radians for roll measurements and aileron deflection, so that it is compatible with system ID models.

**jROLL_KD**

D gain for the custom roll controller. Defined in units of radians for roll measurements and aileron deflection, so that it is compatible with system ID models.

**jBANG_ROLL_LIM**

Roll angle limit for bang-bang controller. If the absolute roll angle exceeds this limit, the bang-bang controller will engage and briefly add an aileron input to return the aircraft to zero roll. Have tested 25 deg in the past.

**jBANG_PITCH_LIM**

Pitch angle limit for bang-bang controller. If the absolute pitch angle exceeds this limit, the bang-bang controller will engage and briefly add an elevator input to return the aircraft to trim pitch. Tested 14 deg in the past.

**jBANG_AMP**

Amplitude of bang-bang controller inputs, expressed as a percentage of 
the maximum deflection. Tested 9% in the past.

# ArduPilot Project

<a href="https://ardupilot.org/discord"><img src="https://img.shields.io/discord/674039678562861068.svg" alt="Discord">

![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master) ![Test Plane](https://github.com/ArduPilot/ardupilot/workflows/test%20plane/badge.svg?branch=master) ![Test Rover](https://github.com/ArduPilot/ardupilot/workflows/test%20rover/badge.svg?branch=master) ![Test Sub](https://github.com/ArduPilot/ardupilot/workflows/test%20sub/badge.svg?branch=master) ![Test Tracker](https://github.com/ArduPilot/ardupilot/workflows/test%20tracker/badge.svg?branch=master)

![Test AP_Periph](https://github.com/ArduPilot/ardupilot/workflows/test%20ap_periph/badge.svg?branch=master) ![Test Chibios](https://github.com/ArduPilot/ardupilot/workflows/test%20chibios/badge.svg?branch=master) ![Test Linux SBC](https://github.com/ArduPilot/ardupilot/workflows/test%20Linux%20SBC/badge.svg?branch=master) ![Test Replay](https://github.com/ArduPilot/ardupilot/workflows/test%20replay/badge.svg?branch=master)

![Test Unit Tests](https://github.com/ArduPilot/ardupilot/workflows/test%20unit%20tests/badge.svg?branch=master)

[![Build SemaphoreCI](https://semaphoreci.com/api/v1/ardupilot/ardupilot/branches/master/badge.svg)](https://semaphoreci.com/ardupilot/ardupilot) [![Build Status](https://dev.azure.com/ardupilot-org/ardupilot/_apis/build/status/ArduPilot.ardupilot?branchName=master)](https://dev.azure.com/ardupilot-org/ardupilot/_build/latest?definitionId=1&branchName=master)

[![Coverity Scan Build Status](https://scan.coverity.com/projects/5331/badge.svg)](https://scan.coverity.com/projects/ardupilot-ardupilot)

[![Autotest Status](https://autotest.ardupilot.org/autotest-badge.svg)](https://autotest.ardupilot.org/)

Ardupilot is the most advanced, full-featured and reliable open source autopilot software available. It has
been under development since 2010 by a diverse team of professional engineers, computer scientists and community contributors.
Our autopilot software is capable of controlling almost any vehicle system imaginable, from conventional
airplanes, quadplanes, multirotors, and helicopters, to rovers, boats, balancebots and even submarines. It is continually being expanded to provide
support for new emerging vehicle types.

## The ArduPilot project is made up of: ##

- ArduCopter: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](https://ardupilot.org/copter/index.html)

- ArduPlane: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](https://ardupilot.org/plane/index.html)

- Rover: [code](https://github.com/ArduPilot/ardupilot/tree/master/Rover), [wiki](https://ardupilot.org/rover/index.html)

- ArduSub : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub), [wiki](http://ardusub.com/)

- Antenna Tracker : [code](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker), [wiki](https://ardupilot.org/antennatracker/index.html)

## User Support & Discussion Forums ##

- Support Forum: <https://discuss.ardupilot.org/>

- Community Site: <https://ardupilot.org>

## Developer Information ##

- Github repository: <https://github.com/ArduPilot/ardupilot>

- Main developer wiki: <https://ardupilot.org/dev/>

- Developer discussion: <https://discuss.ardupilot.org>

- Developer chat: <https://discord.com/channels/ardupilot>

## Top Contributors ##

- [Flight code contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
- [Wiki contributors](https://github.com/ArduPilot/ardupilot_wiki/graphs/contributors)
- [Most active support forum users](https://discuss.ardupilot.org/u?order=post_count&period=quarterly)
- [Partners who contribute financially](https://ardupilot.org/about/Partners)

## How To Get Involved ##

- The ArduPilot project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](https://ardupilot.org/dev/docs/contributing.html)

- We have an active group of Beta Testers to help us improve our code: [release procedures](https://dev.ardupilot.org/wiki/release-procedures)

- Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/ArduPilot/ardupilot/issues).

- Help other users with log analysis in the [support forums](https://discuss.ardupilot.org/)

- Improve the wiki and chat with other [wiki editors on Gitter](https://gitter.im/ArduPilot/ardupilot_wiki)

- Contact the developers on one of the [communication channels](https://ardupilot.org/copter/docs/common-contact-us.html)

## License ##

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://dev.ardupilot.com/wiki/license-gplv3)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

## Maintainers ##

ArduPilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area.

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: Pixhawk, Pixhawk2, PixRacer
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Jacob Walser](https://github.com/jaxxzer):
  - ***Vehicle***: Sub
- [Lucas De Marchi](https://github.com/lucasdemarchi):
  - ***Subsystem***: Linux
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: Batteries
  - ***Subsystem***: GPS
  - ***Subsystem***: Scripting
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash, Tools
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, Rover, AntennaTracker
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Chris Olson](https://github.com/ChristopherOlson):
  - ***Vehicle***: TradHeli
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Eugene Shamaev](https://github.com/EShamaev):
  - ***Subsystem***: CAN bus
  - ***Subsystem***: UAVCAN
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Julien Beraud](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Leonard Hall](https://github.com/lthall):
  - ***Subsystem***: Copter attitude control and navigation
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Pierre Kancir](https://github.com/khancyr):
  - ***Subsystem***: Copter SITL, Rover SITL
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Amilcar Lucas](https://github.com/amilcarlucas):
  - ***Subsystem***: Marvelmind
- [Samuel Tabor](https://github.com/samuelctabor):
  - ***Subsystem***: Soaring/Gliding
