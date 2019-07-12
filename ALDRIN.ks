//ALDRIN Landing Autopilot
//Targetted Landing Autopilot for airless bodies

//Code by Grover - other credits made in code comments
//share and adapt at will :)

//==SETTINGS ARE HERE==

set GateAlt to 1000. //height above landing zone altitude of the approach gate in metres

Set AlignmentTolerance to 0.15. //tolerance for the orbital alignment loop during the orbital phase, in degrees

Set TurnTime to 15. //time taken to rotate the ship from one point to another

set HDGAdjustMult to 2. //multiplier of course error to heading point correction

set HSpdTolerance to 1. //percentage check of groundspeed difference between current and planned to trigger corrective feedback

set AltTolerance to 1. //percentage check of altitude difference between current and planned to trigger corrective feedback

set ThrotMult to 2. //multiplier for throttle feedback, i.e.(at default = 2) if the ship is 2% faster than planned, increase throttle by 4%

set PitchMult to 2. //as above, for pitch feedback

set FinalHSpdTolerance to 0.1. //tolerance for final horizontal speed before touchdown (lower for less stable craft)


FUNCTION StatusText //function to place StatusText messages where they belong
{
	PARAMETER TEXT.
	PRINT TEXT AT (9,2). //9,2 is the position after "StatusText: "
}.

function circle_bearing //function by TDW89 for calculating the Great Circle Bearing from P1 to P2
{
	parameter
	p1, //...this point...
	p2. //...to this point.

	return mod(360+arctan2(sin(p2:lng-p1:lng)*cos(p2:lat),cos(p1:lat)*sin(p2:lat)-sin(p1:lat)*cos(p2:lat)*cos(p2:lng-p1:lng)),360).
}.

function circle_destination //Function by TDW89 for calculating the position of a point a certain bearing and sistance from a point
{
 parameter
  p1,     //...this point...
  b,      // ...with this as your intitial bearing...
  d,      // ...for this distance...
  radius IS SHIP:BODY:RADIUS. // ...around a sphere of this radius.

 local lat is arcsin(sin(p1:lat)*cos((d*180)/(radius*constant:pi))+cos(p1:lat)*sin((d*180)/(radius*constant:pi))*cos(b)).
 local lng is 0.
 if abs(Lat) <> 90 {
  set lng to p1:lng+arctan2(sin(b)*sin((d*180)/(radius*constant:pi))*cos(p1:lat),cos((d*180)/(radius*constant:pi))-sin(p1:lat)*sin(lat)).
 }.
 
 return latlng(lat,lng).
}.

function circle_distance //Function by TDW89 to calculate the distance around a body from one point to the other
{
 parameter
  p1,     //...this point...
  p2,     //...to this point...
  radius IS SHIP:BODY:RADIUS. //...around a body of this radius.
 local A is sin((p1:lat-p2:lat)/2)^2 + cos(p1:lat)*cos(p2:lat)*sin((p1:lng-p2:lng)/2)^2.
 
 return radius*constant:PI*arctan2(sqrt(A),sqrt(1-A))/90.
}.

function vel_bearing //Function by TDW89 to calculate the heading of the Surface Velocity Vector
{
    local trig_x is vdot( heading( 90, 0 ):vector, ship:srfprograde:vector ).
    local trig_y is vdot( heading(  0, 0 ):vector, ship:srfprograde:vector ).

    return mod( arctan2( trig_x, trig_y ) +360, 360 ).
}

function ThrustOutput //function to calculate the thrust output of a given throttle setting
{
	parameter MaxThru. //parameter for max thrust of engines
	parameter MinThro. //Parameter for Min throttle setting of engines
	Parameter Thro. //parameter for throttle setting to evaluate
	
	set MinThru to MaxThru * MinThro. //calculate minimum possible throttle
	set ThruAdd to (MaxThru - MinThru) * Thro. //calculate the thrust made up by the throttle setting
	set Thru to MinThru + ThruAdd.
	
	return Thru.
}

function ThrottNeeded //function to calculate the throttle setting needed for a given thrust (required when minimum throttle isn't 0%)
{
	parameter Thru. //thrust required
	parameter MinThro. //minimum throttle setting of engines
	parameter MaxThru. //ships max thrust
	
	set MinThru to MaxThru * MinThro. //calculate minimum possible thrust
	set ThruRange to MaxThru - MinThru. //calculate the difference between max thrust and min thrust
	set ThruAdd to Thru - MinThru. //calculate the thrust make up needed with throttle
	set Thro to ThruAdd / ThruRange. //calculate ratio between thrust needed from throttle and thrust range
	
	return Min(1,MAX(0.001,Thro)). //return Throttle setting, forced within acceptable throttle range to avoid engine shutdown
}

function OrbitLift //Function to calculate the phantom lift experienced when at a significant fraction of orbital speed, expressed as the acceleration. the vertical accleration required by thrusting, to maintain vertical speed is equal to the Acceleration of gravity, less this figure
{
	Parameter Spd. //orbital speed, does not allow for the difference between orbital speed and ground speed, as the surface speed of the moon's surface is less than 5m^s
	Parameter Alti. //altitude
	
	set Accel to (Sqrt(spd^2 + (Alti + ship:body:radius)^2) - Alti - ship:body:radius) * 2.
	
	Return Accel.
}

function FlightPlan //function to create a flight plan of the descent by stepwise simulation
{
	parameter MinThrottle.
	parameter GateAlt. //altitude of the approach gate above "sea level"
	parameter ShipThrust is ship:maxthrust. //max thrust of ship, default is current max thrust
	parameter PlanAlt is Ship:ORBIT:PERIAPSIS. //altitude at the start of the descent burn, default is ship periapsis
	parameter VSpd is 0. //Vertical Speed at start of descent burn, default is 0
	parameter PlanHSpd is Ship:GroundSpeed. //horizontal speed at start of descent burn, default is surface frame speed at time of calculation
	parameter ShipMass is Ship:Mass. //mass of ship at start of burn, default is current ship mass
	parameter ShipISP is Ship:ISP. //ISP of the ships engines
	
	set PlanThru to ThrustOutput(ShipThrust,MinThrottle,0.95). //calculate the thrust generated at 95% throttle, using existing function, 5% lower thrust allows for anomalies and inaccuracies later.
	Set Dist to 0. //distance to be tracked, to act as a trigger for descent burn commencement later
	set StartAlt to PlanAlt.
	
	set Plan to List().
	Plan:add(Dist,PlanAlt,PlanHSpd,Vspd). //add starting figures to flightplan in index position 0.
	
	until PlanHSpd < 0 //loop that runs until the horizontal speed is arrested. First run of the stepwise simulation, little altitude data
	{
		set Gravity to (ship:body:mu * ShipMass) / (PlanAlt + Ship:Body:radius)^2. //gravity force acting on the ship
		set LiftNeeded to Gravity - (OrbitLift(PlanHSpd,PlanAlt) * ShipMass). //calculate the force needed by the engines to maintain vertical speed
		set PlanPitch to arcsin(LiftNeeded/PlanThru). //calculate the pitch angle needed to maintain vertical speed at planned thrust
		set HAcc to (cos(PlanPitch) * PlanThrust) / ShipMass. //calculate the horizontal Acceleration produced at the profile calculated above
		set Vspd to Vspd - LiftNeeded / ShipMass. //calculate what the new vertical speed would be if not pitched up
		set PlanHSpd to PlanHSpd - HAcc. //reduce Horizontal Speed by acceleration this pass
		set Dist to Dist + PlanHSpd. //increase distance by current speed
		set PlanAlt to PlanAlt + VSpd. //Decrease Altitude by vertical speed (NOTE the VSpd variable will be negative during descent)
		set ShipMass to ShipMass - ((PlanThru/1000)/(constant:g0 * ShipISP)). //reduce ship mass by the amount of fuel used per second at the given thrust
		
		set NewEntry to LIST(Dist,PlanAlt,PlanHSpd,Vspd). //list the parameters to be stored in the plan
		Plan:add(NewEntry). //add the entry to the plan
	}
	
	//iterate over plan to find the vertical speed that arrives at the gate in time
	from {local i is 1.} until HandoverAlt > 0 step {set i to i+1.} do
	{
		if abs(Plan[i][3])*(Plan:Length-1-i) > Plan[i][1] - GateAlt //if the vertical speed at this entry gives a total altitude change (over the remaining duration of the plan) greater than the altitude between current and final alt
		{
			set PitchTime to i. //remember the index of the handover point (handover between pure braking and controlled descent)
			set HandoverAlt to Plan[i][1]. //remember the handover altitude
			set Plan[i][4] to "HANDOVER". //set a flag at this entry to act as a trigger between pure braking and controlled descent
			break. //break the loop before the value is overwritten by each subsequent speed.
		}
	}
	
	//invert distance variables of all elements in the FlightPlan to represent distance to LZ, rather than from the start
	set DescentDistance to Plan[Plan:Length-1][0]. //remember the last distance value recorded
	from {local i is 0.} until i = Plan:Length-1 step {set i to i+1.} do
	{
		set Plan[i][0] to abs(Plan[i]-DescentDistance).
	}
	
	//find the FlightPlan entry that lands at the gate
	from {local i is 0.} until i = Plan:Length-1 step {set i to i+1.} do
	{
		if Plan[i][0] < 1000
		{
			set GateEntry to i.
			set Plan[i][4] to "GATE". //set a flag to act as a trigger to change to approach mode
			break. //break the loop before the value is overwritten by each subsequent entry
		}
	}
	
	set ControlledTime to GateEntry-PitchTime. //remember how long the flightplan calls for controlled descent (how many steps are after the handover time)
	set TgtVSpd to (HandoverAlt-GateAlt)/ControlledTime. //recalculate Target Vertical Speed for Controlled Descent to finish at exactly the correct altitude.
	
	//regenerate flight plan with correct vertical speed for the controlled part of the descent
	from {local t is i+1.} until t = Plan:length-1 step {set t to t+1.} do
	{
		set Plan[t][1] to Plan[t-1][1]-TgtVSpd.
	}
	//FlightPlan now has two stages, pure braking from the start, transitioning to controlled descent, maintaining the appropriate descent speed to arrive at the gate at the correct altitude.

	
	Return Plan.
}

//Initialisation
//set up terminal, get landing target information, check for vessel stats loaded, check orbit params
SET TERMINAL:WIDTH to 36.
SET TERMINAL:HEIGHT to 20.
SET INIT TO FALSE.
CLEARSCREEN.
PRINT "========ALDRIN========" AT (0,0). //pretty terminal header
PRINT "StatusText: " AT (0,2). //StatusText line to be maintained, print StatusText messages up to 26 characters starting at (9,2)

if UseRCS = TRUE
{
	RCS ON.
}

until INIT = TRUE
{
set RUNMODE to "ORB".

if ALDRINSTATS = TRUE
{
	
}
else
{
	set RUNMODE to "PROBLEM".
	StatusText("SHIP STATS NOT LOADED").
	break.
}

if SHIP:ORBIT:PERIAPSIS < 30000
{
	set RUNMODE to "PROBLEM".
	StatusText("PE MUST BE ABOVE 30K").
	break.
}

if SHIP:ORBIT:APOAPSIS > 40000
{
	set RUNMODE to "PROBLEM".
	StatusText("AP MUST BE BELOW 40K").
	break.
}

set WayPoints to ALLWAYPOINTS().

if WayPoints:CONTAINS("ALDRIN LZ") = TRUE //search for KSP Waypoint named "ALDRIN LZ"
{
	set LZ to WAYPOINT("ALDRIN LZ"):GEOPOSITION.
	set LZSOURCE to "WAYPOINT".
	PRINT "LZ set to WAYPOINT" at (0,4).
}
ELSE IF TARGET:StatusText = "LANDED" OR "SPLASHED" //2nd option, if a landed ship or flag is selected
{
	set LZ to TARGET:GEOPOSITION.
	set LZSOURCE to "TARGET".
	PRINT "LZ set to TARGET" at (0,4).
}
ELSE //if no other targets, use ships position in 90 degrees
{
	set LZ to circle_destination(ship:geoposition,vel_bearing(),SHIP:BODY:RADIUS * constant:pi * 0.5). //trace around the surface by the current surface velocity heading for 1/4 the body's circumference
	set LZ:LNG to LZ:LNG - (360 * ship:orbit:period * 0.3 /ship:body:rotationperiod). //reduce the longitude by how far the body will rotate in the approximate time of landing
	if lz:lng < -180
	{
		set lz:lng to lz:lng + 360. //if longitude was nudged to an improper value, bring it back round to a proper one
	}
	set LZSOURCE to "NONE".
	PRINT "LZ set by Current Pos" at (0,4).
}

print "LZ LAT: " + ROUND(LZ:LAT,4) at (0,5).
print "LZ LNG: " + ROUND(LZ:LNG,4) at (0,6).

if RUNMODE = "PROBLEM"
{
	print "ERROR TERMINATED PROGRAM" at (0,19).
}
ELSE
{
	set RUNMODE to "ORB".
}
SET INIT TO TRUE.
}

//runmode Orb
//orbital mode, warp to adjustment node (90 degrees before target) and conduct Landing Target alignment
//calculate and store descent flight plan
	//stepwise simulation of descent
	//phase 1
		//maximum braking force (at 98% throttle), allow for generating lift to maintain vertical speed
		//track vessel mass to update TWR, vertical speed if not controlled, allow for phantom lift at high speed
		//calculate Horizontal distance travelled during braking
		//set gate altitude, determine target vertical speed (which "step's" vertical speed arrives at the gate's altitude on time)
	//phase 2
		//re-run simulation, two phases, braking (pitch = 0) and descending (Vspeed = constant)
		//record distance to target, horizontal speed, altitude as 2D list

if RUNMODE = "ORB"
{
	StatusText("INIT COMPLETE").
	if LZSOURCE = "NONE" //if LZ was earned from ship's position, skipt to next stage
	{
		SET OrbitAligned to TRUE.
	}
	set NodeLNG to lz:lng - 90. //make adjustment node longitude 90 degrees west of LZ
		
	if ship:orbit:inclination > 90 //if orbit is retrograde, move longitude to 90 degrees east
	{
		set NodeLNG to NodeLNG + 180.
	}
	if ship:orbit:inclination < -90 //if orbit is retrograde, move longitude to 90 degrees east
	{
		set NodeLNG to NodeLNG + 180.
	}
	
	if NodeLNG < -180
	{
		set NodeLNG to NodeLNG + 360. //if the adjustment node is illegal, fix it.
	}
	if NodeLNG > 180
	{
		set NodeLNG to NodeLNG - 360. //if the adjustment node is illegal, fix it.
	}
	
	UNTIL OrbitAligned = TRUE
	{
		StatusText("WARPING TO ADJUSTMENT").
		SET KUNIVERSE:TIMEWARP:WARP to 3. //50x timewarp to adjustment node
		until SHIP:GEOPOSITION:LNG < NodeLNG + 0.5 AND SHIP:GEOPOSITION:LNG > NodeLNG - 0.5 //until ship is within 0.5 degrees of adjustment node longitude
		{
			wait 0. //do nothing
		}
		Set Warp to 0.
		//ship is now at orbital adjustment node
		
		//adjust heading to correct course
		
		Lock CrsError to vel_bearing()-circle_bearing(ship:geoposition,LZ).
		
		until abs(CrsError) < AlignmentTolerance
		{
			if CrsError > 0 //ship's course bearing is greater than required,
			{
				lock HDGPoint to vel_bearing()-90.
			}
			else
			{
				lock HDGPoint to vel_bearing()+90.
			}
			lock steering to heading(HDGPoint,0).
			wait TurnTime.
			
			if ULLAGE = TRUE //if ullage is needed
			{
				Set Ship:Control:FORE to 1.
				Wait 1.
			}
			
			set throttle to 1. //ignite engines to correct course
			Set Ship:Control:Fore to 0.
		}
		
		set throttle to 0. //cut engines
		SET OrbitAligned TO TRUE.
		StatusText("Orbital Plane Aligned").
	}
	
	Set GateAlt to LZ:TERRAINHEIGHT + GateHeight. //calculate the altitude of the approach gate
	Set Plan to FlightPlan(MinThrottle,GateAlt,ShipThrust). //run the function to create a flight plan for the descent burn, not using totaly accurate starting data yet.
	StatusText("Flight Plan Generated").
	
	//warp to near braking burn start
	SET KUNIVERSE:TIMEWARP:WARP to 3. //50x timewarp to braking burn start
	until circle_distance(Ship:geoposition,LZ) < Plan[0][0]+(ship:groundspeed*10*TurnTime) //until ship is within the distance travelled in 10 turn times of the braking start
	{
		wait 0.
	}
	SET KUNIVERSE:TIMEWARP:WARP to 0.
	
	//recalc flightplan with accurate starting data
	Set Plan to FlightPlan(MinThrottle,GateAlt,ShipThrust,ship:altitude,ship:verticalspeed).
	
	//set up display for descent information
	print "      CURRENT    PLANNED" 			at (0,8). //header line
	print "DIST " 	at (0,9).
	print "HSPD "	at (0,10).
	print "VSPD "	at (0,11).
	print "ALT  "	at (0,12).
	print "CRS  "	at (0,13).
	print "HCOR "	at (0,14).
	print "PIT  "	at (0,15).
	
	lock steering to heading(circle_bearing(LZ,Ship:geoposition),0). //lock the steering to the initial direction, heading away from target at 0 degrees pitch.
	
	set RUNMODE to "BRAKE".
}

//runmode brake
//engage engines for descent, burn to follow flight plan
	//start by braking at 0* pitch, heading adjustments by simply offsets
	//once desired vertical speed is achieved, set default control inputs to maintain vertical speed (HDG remains unchanged)
	//simple offsets in place to correct for hi/lo altitude and speed for set distance to target
	//aim for "Gate" at 1km before and 1km above landing target
	
if RUNMODE = "BRAKE"
{
	until circle_distance(Ship:geoposition,LZ) <= FlightPlan[0][0] //wait until distance to LZ is equal to descent plan distance
	{
		wait 0.
	}
	
	if ULLAGE = TRUE
	{
		set Ship:Control:Fore to 1.
		Wait 1.
	}
	
	StatusText("Braking Burn").
	set PlanThrot to 0.95.
	set ThrotAdjust to 0.
	lock throttle to max(0.001,min(1,PlanThrot + ThrotAdjust)). //throttle lock calculation, throttle setting bracketed between 0.001 and 1 to prevent engine shutdown
	Set Ship:Control:fore to 0.
	
	lock PlanHDG to Circle_bearing(LZ,Ship:geoposition). //lock default heading to point directly away from LZ on the compass
	set HDGAdjust to 0.
	
	set PlanPitch to 0.
	Set PitchAdjust to 0.
	
	lock steering to heading(PlanHDG+HDGAdjust,max(0,min(88,PlanPitch+PitchAdjust))). //pitch brakceted between 0 and 88
	lock LZRange to circle_distance(Ship:Geoposition,LZ).
	
	set Entry to 0.
	set GATE to FALSE. //flag to kick out of controlled descent
	
	until GATE = TRUE //loop runs for the duration of the braking burn
	{		
		if LZRange < Plan [Entry+1][0] //if the next entry in the flight plan is reached
		{
			set Entry to Entry+1. //move to next entry
		}
		
		if Plan[Entry][4] = "HANDOVER" //if program is at the entry where controlled descent should begin
		{
			set Controlled to TRUE.
			StatusText("Controlled Descent").
		}
		
		//print all relevant data points to output
		print "DIST " at (0,9).  print round((LZRange/1000),2) at (6,9). print " " + Round((Plan[Entry][0]/1000),2) at (16,9). Print " KM" at (34,9).
		print "HSPD " at (0,10). print round(ship:groundspeed,1) at (6,10). print " " + round(Plan[Entry][2],1) at (16,10). print " M/S" at (33,10).
		print "VSPD " at (0,11). print round(ship:verticalspeed,1) at (6,11). print " " + round(Plan[Entry][3],1) at (16,11). print " M/S" at (33,11).
		print "ALT  " at (0,12). print round(ship:altitude/1000,1) at (6,12). print " " + Round (Plan[Entry][1],1) at (16,12). print " KM" at (34,12).
		print "CRS  " at (0,13). print round(Vel_bearing(),1) at (6,13). print " " + round(circle_bearing(ship:geoposition,LZ),1) at (16,13). print " DEG" at (33,13).
		print "HCOR " at (0,14). print round(HDGAdjust,1) at (6,14). print " DEG" at (33,14).
		print "PIT  " at (0,15). print " " + round(PlanPitch+PitchAdjust,1) at (16,15).
		
		//course error correction
		if vel_bearing < 180 //for prograde orbit
		{
			set HDGAdjust to (vel_bearing()-circle_Bearing(Ship:geoposition,LZ)) * HDGAdjustMult.
		}
		else //retrograde orbit
		{
			set HDGAdjust to (Circle_bearing(Ship:geoposition,LZ)-vel_bearing()) * HDGAdjustMult.
		}
		
		set throtAdjust to 0. //reset throttle adjustment to zero to be recalculated below this pass
		set PitchAdjust to 0. //reset pitch adjustment to zero to be recalculated below this pass
		
		if Controlled = TRUE //only run the controlled descent portion if passed handover point
		{
			set Gravity to (ship:body:mu * ship:Mass) / (ship:altitude + Ship:Body:radius)^2. //gravity force acting on the ship
			set LiftNeeded to Gravity - (OrbitLift(ship:groundspeed,ship:altitude) * Ship:Mass). //calculate the force needed by the engines to maintain vertical speed
			set PlanPitch to arcsin(LiftNeeded/ThrustOutput(ship:maxthrust,MinThrottle,0.95)). //calculate the pitch angle needed to maintain vertical speed at planned thrust
			
			//HSpd feedback
			set HSpdCheck to 100 - (ship:groundspeed/Plan[entry][2]) * 100.
			if abs(HSpdCheck) > HSpdTolerance
			{
				if HSpdCheck > 0 //ship is going faster than planned
				{
					set ThrotAdjust to HSpdCheck * ThrotMult.
				}
				else //ship is going slower than planned
				{
					set PitchAdjust to HSpdCheck * PitchMult.
				}
			}
			
			//ALT feedback
			set ALTCheck to 100 - (ship:altitude/Plan[entry][1]) * 100.
			if abs(ALTCheck) > AltTolerance
			{
				if ALTCheck > 0 //current altitude is too high
				{
					if HSpdCheck > 0 //speed is above planned
					{
						set PitchAdjust to -AltCheck * PitchMult.
					}
					else //speed is below planned
					{
						set ThrotAdjust to -AltCheck * ThrotMult.
					}
				}
				else //current altitude is too low
				{
					if HSpdCheck > 0 //speed is above planned
					{
						//do nothing to pitch
					}
					else //speed is below planned
				{
						set ThrotAdjust to AltCheck * ThrotMult.
					}
				}
			}
		}
		if Plan[Entry][4] = "GATE"
		{
			Set GATE to TRUE. //flag to kick out of brake and into approach mode
			StatusText("FINAL APPROACH").
		}
	}
	
	set RUNMODE to "APPROACH".
}

//runmode approach
//forgo approach to target in favour of arresting horizontal speed (lock steering to surface retro)
//maintain safe descent speed until altitude is 50m
	//lock steering to up, and trim horizontal speed with RCS translation
	//touchdown, make ship safe, deploy panels etc.
	//print stats
	
if RUNMODE = "APPROACH"
{
	LEGS ON. //deploy landing gear
	until FINISH = TRUE
	{
		lock steering to SRFRETROGRADE.
		Set ThrotPID to pidloop(0.5,0.1,0.3,0.001,1).
		set ThrotPID:Setpoint to (ALT:Radar/20)*-1. //try to maintain a descent rate equal to 5% the radar altitude, i,e, 25m/s at 500m alt.
		set throttle to ThrotPID:update(time:seconds,ship:verticalspeed). //update throttle using PID loop.
		until ALT:radar <= 50
		{
			set ThrotPID:Setpoint to (ALT:Radar/20)*-1. //try to maintain a descent rate equal to 5% the radar altitude, i,e, 25m/s at 500m alt.
			set throttle to ThrotPID:update(time:seconds,ship:verticalspeed). //update throttle using PID loop.
			wait 0.
		}
		set FinHeading to vel_bearing().
		lock steering to heading(FinHeading,90).
		
		//use RCS to trim horizontal speed
		until ship:groundspeed < FinalHSpdTolerance
		{
			set ship:control:top to 1. //rcs thrusters in translation, relies upon ships roll pointing this direction retrograde.
		}
		set ship:control:top to 0.
		
		StautsText("Final Descent").
		
		//wait for landing
		until ship:status = "LANDED" or "SPLASHED"
		{
			set ThrotPID:Setpoint to (ALT:Radar/20)*-1. //try to maintain a descent rate equal to 5% the radar altitude, i,e, 25m/s at 500m alt.
			set throttle to ThrotPID:update(time:seconds,ship:verticalspeed). //update throttle using PID loop.
			wait 0.
		}
		
		//release all controls
		unlock steering.
		unlock throttle.
		set throttle to 0.
		StatusText("TOUCHDOWN, MECO").
		
		wait 1.
		
		//print final stats
		
		//deploy panels etc.
		if ManagePanels = TRUE
		{
			panels on.
			StatusText("Deploying Solar Panels").
		}
	}
}