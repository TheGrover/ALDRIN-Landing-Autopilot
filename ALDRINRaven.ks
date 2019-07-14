//ALDRIN Autopilot Example Config File

Set ALDRINStats to True. //general flag to be read by the main program to confirm this file is loaded.

Set ManagePanels to TRUE. //Should the autpilot manage solar panels?

Set Ullage to FALSE. //Do the ship's Engines require Ullage?

Set EmptyMass to 659. //Dry Mass of the ship (in Kg), after all fuel is depleted, including any return/ascent stages and payload
//I.E. the total ship mass less this figure should give the mass of available fuel

Set ShipThrust to 1.386. //Maximum Thrust of the ship, using all engines

Set ShipISP to 287. //ISP of the ship's engines, in seconds

Set MinThrottle to 0.29. //minimum throttle of the engines as a ratio (I.E. 17% throttle is 0.17)

Set UseRCS to TRUE. //are RCS thrusters required for adequate control?