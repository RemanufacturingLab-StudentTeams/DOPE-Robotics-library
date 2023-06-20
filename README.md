# Project Clemex Assembly
 
## Overview
Normally a Doosan cobot is programmed with the software developed by Doosan (DART-studio). This software however is behind a paywall. This library is developed with freeware in mind. That is why this library has been made with DRL-Studio.

The goal of the DOPE library is to simplify future development of code for Doosan cobots. Functions in this library are meant to be as generic as possible so that they can be reused by other users for their own applications. The DOPE library is meant to be expanded by future contributions.

The DOPE library is specifically meant to be used with the Doosan cobots. However some of the mathematics performed by the functions in this library might be useful for other robotics applications.

## Used technology
As mentioned before the DRL-Studio was used for the development of this library. For the Doosan cobot to work with DRL-Studio some additional steps are needed. For example a Homberger hub needs to be available. The Homberger hub is the link between the Doosan cobot and DRL-studio.

The programming language for this library is python 3.8.

## Install
1. Download dope.py. 
2. Create library folder in DRL-studio. 
3. Add the dope.py to the desired library folder. 
4. In your main file use import dope as dp.

## Dependencies
In combination with DRL-studio, multiple libraries were also used:

- DRCF - 2.10 (This is a library from DRL-Studio itself) 
- Motion lib - 1.3.3 (internal in DRL-Studio using powerup library function)
- Numpy - 1.24.2
