# ALDRIN-Landing-Autopilot
Landing Autopilot for KSP written for the KOS Autopilot System

FILES:
ALDRIN.KS - 
  Main program, containing all relevant functions in one package to carry out the descent
  
ALDRINRaven.KS -
  Example configuration file, to be run before the main program to store some vessel-specific values to be used.
  Running the main program without a configuration file for the ship will cause an early termination.
  Each different ship must have a bespoke configuration file loaded before main program initialisation
