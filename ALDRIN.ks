//ALDRIN Landing Autopilot
//Targetted Landing Autopilot for airless bodies

//Code by Grover - other credits made in code comments
//share and adapt at will :)

//==SETTINGS ARE HERE==

set OriginalIPU to Config:IPU. //remember the original IPU to reset at the end of the program
set config:IPU to 2000. //forces KOS to use the maximum Instructions per tick (needed to calculate variables in time)

set Config:DEFAULTFONTSIZE to 14.

set GateHeight to 1000. //height above landing zone altitude of the approach gate in metres

set GateOffset to 2000. //distance on plan between LZ and gate

Set AlignmentTolerance to 0.05. //tolerance for the orbital alignment loop during the orbital phase, in degrees

Set TurnTime to 10. //time taken to rotate the ship from one point to another

set LevelOffDist to 10000. //distance from the target to begin reducing vertical speed

set HDGAdjustMult to 3. //multiplier of course error to heading point correction (i.e. if course error is 1 deg. point the ship 1 * Mult off the retrograde to correct

set HSpdTolerance to 0.2. //percentage check of groundspeed difference between current and planned to trigger corrective feedback

set AltTolerance to 0.2. //percentage check of altitude difference between current and planned to trigger corrective feedback

set PitchMult to 3. //multiplier for Pitch feedback, i.e.(at default = 3) if the ship is 5% faster than planned, decrease Pitch by 15 degrees

set ThrotMult to 3. //As Above, for Throttle

set VSpdMult to 10. //increases the sensitivity of the Vertical Speed Feedback, higher numbers make Vertical Speed affect pitch/throttle more

set MaxHdgCorr to 5. //maximum heading correction (yaw away from "retrograde") in degrees

set MaxPitchCorr to 20. //maximum pitch correction for course correction in degrees

set HSpdMult to 1. //multiplier for the Horizontal Speed feedback to descent guidance

set AltMult to 5. //multiplier for the Altitude Feedback to descent guidance

set FinalHSpdTolerance to 0.1. //tolerance for final horizontal speed before touchdown (lower for less stable craft)

set AltCheck to 0.
set HSpdCheck to 0.
set VSpdCheck to 0.


FUNCTION StatusText //function to place StatusText messages where they belong
{
	PARAMETER TEXT.
	PRINT "                            " AT (8,2).
	PRINT TEXT AT (8,2). //9,2 is the position after "StatusText: "
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
	
	set MinThru to MaxThru * MinThro. //calculate minimum possible thrust
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
	
	Return Accel. //returns the phantom acceleration (M/s^2 caused by travelling near or above orbital velocity, forced not to return a negative value
}

function FlightPlan //function to create a flight plan of the descent by stepwise simulation
{
	parameter MinThrottle.
	parameter GateAlt. //altitude of the approach gate above "sea level"
	parameter ShipThrust. //max thrust of ship
	parameter PlanAlt. //altitude at the start of the descent burn
	parameter VSpd. //Vertical Speed at start of descent burn
	parameter PlanHSpd. //horizontal speed at start of descent burn, default is surface frame speed at time of calculation
	parameter ShipMass. //mass of ship at start of burn
	parameter ShipISP. //ISP of the ships engines
	
	//RESOLUTION SETTING: this is the "Step size" of the descent simulation. smaller is more accurate and precise, but makes the plan generation time take longer (and require more memory)
	Set Resolution to 1.
	
	set PlanThru to ThrustOutput(ShipThrust,MinThrottle,0.95). //calculate the thrust generated at 95% throttle, using existing function, 5% lower thrust allows for anomalies and inaccuracies later.
	Set Dist to 0. //distance to be tracked, to act as a trigger for descent burn commencement later
	set StartAlt to PlanAlt.
	
	set Plan to List().
	Plan:add(list(Dist,PlanAlt,PlanHSpd,Vspd)). //add starting figures to flightplan in index position 0.
	set DescentError to FALSE.
	
	until PlanHSpd < 0 //loop that runs until the horizontal speed is arrested. First run of the stepwise simulation, little altitude data
	{
		set Gravity to (ship:body:mu * (ShipMass*1000)) / (PlanAlt + Ship:Body:radius)^2. //gravity force acting on the ship in Newtons
		set LiftNeeded to Gravity - (OrbitLift(PlanHSpd,PlanAlt) * (ShipMass*1000)). //calculate the force needed by the engines to maintain vertical speed in Newtons
		set PitchRatio to LiftNeeded/(PlanThru*1000).
		if PitchRatio > 1 //if more thrust is required to maintain VSpd than available
		{
			clearscreen.
			print "ERROR: Ship is unable to decelerate".
			print "without losing altitude control".
			print " ".
			print "Redesign ship to have higher TWR!".
			print "Program will terminate".
			set DescentError to TRUE.
			break.
		}
		else
		{
			set PlanPitch to arcsin(PitchRatio). //calculate the pitch angle needed to maintain vertical speed at planned thrust
		}
		
		set HAcc to (cos(PlanPitch) * (PlanThru*1000)) / (ShipMass*1000). //calculate the horizontal Acceleration produced at the profile calculated above (In m/s^2)
		set VAcc to (LiftNeeded / (ShipMass*1000)). //calculate vertical acceleration (acceleration due to gravity)
		set Vspd to Vspd - (VAcc * Resolution). //calculate what the new vertical speed would be if not pitched up
		set PlanHSpd to PlanHSpd - (HAcc * Resolution). //reduce Horizontal Speed by acceleration this pass
		set Dist to Dist + (PlanHSpd * Resolution). //increase distance by current speed
		set PlanAlt to PlanAlt + (VSpd * Resolution). //Decrease Altitude by vertical speed (NOTE the VSpd variable will be negative during descent)
		set FuelRate to ((PlanThru*1000)/(constant:g0 * ShipISP) * Resolution). //calculate the fuel flow rate of the main engines in KG/step
		set ShipMass to ShipMass - (FuelRate/1000). //reduce ship mass by the amount of fuel used per second at the given thrust, multiplied by the resolution
		set NewEntry to LIST(Dist,PlanAlt,PlanHSpd,(Vspd/Resolution)). //list the parameters to be stored in the plan
		Plan:add(NewEntry). //add the entry to the plan
		
		//DEBUG ZONE
		//print "HSPD: " + PlanHSpd.
		//print "Vspd: " + Vspd.
		//print "LIFT Rat: " + PitchRatio.
		//print "LiftNeeded:" + LiftNeeded.
	}
	if DescentError = TRUE //if an error was encountered
	{
		set PlanF to list(list(-10)). //make a very small flightplan with a negative first distance
	}
	else
	{
		set PlanF to List(). //create a new empty plan to be copied into with values corrected and finalised.
		
		//invert distance variables of all elements in the FlightPlan to represent distance to LZ, rather than from the start
		set DescentDistance to Plan[Plan:Length-1][0]. //remember the last distance value recorded
		set Distances to list(). //create a new list for distances
		
		from {local i is 0.} until i = Plan:Length-1 step {set i to i+1.} do
		{
			set NewDist to abs(plan[i][0] - DescentDistance). //invert the distance of this element in the plan
			Distances:ADD(NewDist). //add the new distance to the new list	
		}
		
		//find the FlightPlan entry that lands at the gate
		Set GateDistRaw to DescentDistance - GateOffset. //calculate the total distance travelled in the flightplan minus 1km
		from {local i is 0.} until i = Plan:Length-1 step {set i to i+1.} do
		{
			if Plan[i][0] > GateDistRaw
			{
				set GateEntry to i.
				set GateASL to Plan[i][1].
				set FlagG to "GATE". //create a flag to store in the plan
				Plan[i]:add(FlagG). //set a flag to act as a trigger to change to approach mode
				break. //break the loop before the value is overwritten by each subsequent entry
			}
		}
	
		//iterate over plan to find the vertical speed that arrives at the gate in time
		set HandoverAlt to -1.
		from {local i is 0.} until HandoverAlt > 0 step {set i to i+1.} do
		{
			if (abs(Plan[i][3])*((GateEntry-i) * Resolution)) > (Plan[i][1] - GateAlt) //if the vertical speed at this entry gives a total altitude change (over the remaining duration until the gate) greater than the altitude between current and gate alt
			{
				set PitchTime to i. //remember the index of the handover point (handover between pure braking and controlled descent)				
				set HandoverAlt to Plan[i][1]. //remember the handover altitude
				set FLAG to "HANDOVER".//create a flag to store in the plan
				Plan[i]:add(FLAG). //set a flag at this entry to act as a trigger between pure braking and controlled descent
				break. //break the loop before the value is overwritten by each subsequent entry.
			}
		}
			
		set ControlledTime to (GateEntry-PitchTime) * Resolution. //remember how long the flightplan calls for controlled descent (how many steps are after the handover time)
		set TgtVSpd to (HandoverAlt-GateAlt)/ControlledTime * -1. //recalculate Target Vertical Speed for Controlled Descent to finish at exactly the correct altitude (Negative because we are descending)
		
		set AltList to list(). //create a new list to store new altitude information
		
		//iterate over the flightplan to correct vertical speed and altitude data
		from {local i is 0.} until i = Plan:Length-1 step {set i to i+1.} do
		{
			if plan[i][3] > TgtVSpd //if the target descent rate hasn't been reached yet
			{
				set NewAltList to list(Plan[i][1],Plan[i][3]). //make a copy of the original plan's altitude and VSpd
				AltList:Add(NewAltList). //store these values in the new list			
			}
			else //we are at or beyond the point of controlled descent start
			{				
				set NewAlt to (AltList[i-1][0] + (TgtVSpd * Resolution)). //set new altitude to previous altitude minus vertical speed (Speed here is negative)
				set NewAltList to list(NewAlt,TgtVSpd). //create a new list of the new altitude and target vertical speed
				AltList:Add(NewAltList). //add this sublist to the new altitude list
			}
		}
		
		//regenerate flight plan with correct vertical speed for the controlled part of the descent
		from {local t is 0.} until t = Plan:length-1 step {set t to t+1.} do
		{
			set NewEntry to list(Distances[t],AltList[t][0],Plan[t][2],AltList[t][1]). //make a new sublist from all the elements recalculated above
			PlanF:add(NewEntry). //add the new sublist to the flightplan
		}
		//FlightPlan now has two stages, pure braking from the start, transitioning to controlled descent, maintaining the appropriate descent speed to arrive at the gate at the correct altitude.
		
		//place the handover flag into the new FlightPlan
		set FLAG to "HANDOVER".
		PlanF[PitchTime]:add(FLAG).
		//place the Gate flag into the new flightplan
		PlanF[GateEntry]:add(FlagG).
	}

	Return PlanF.
}

//Initialisation
//set up terminal, get landing target information, check for vessel stats loaded, check orbit params
SET TERMINAL:WIDTH to 36. //return to 36 later
SET TERMINAL:HEIGHT to 20. //return to 20 later

set ClearLine to "                              ". //a string of spaces to act as a line clearer

set SOUND to GetVoice(0). //get the voice to play sounds later
set DoubleBeep to list(NOTE(1000,0.1),NOTE(0,0.2),NOTE(1000,0.1)).

SET INIT TO FALSE.
CLEARSCREEN.
PRINT "========ALDRIN========" AT (0,0). //pretty terminal header
PRINT "Status: " AT (0,2). //StatusText line to be maintained, print StatusText messages up to 26 characters starting at (9,2)

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

set WayPoints to ALLWAYPOINTS(). //make a list of all the KSP waypoints available
set WaypointFound to FALSE.
set CheckTarget to FALSE.

for item in WayPoints //iterate over the list of waypoints to...
{
	if item:name = "ALDRIN LZ" //find any waypoints with the correct name to be used to land at
	{
		set WaypointFound to TRUE.
	}
}

if HasTarget = TRUE //check if the ship has a vessel or flag targetted
{
	Set CheckTarget to True.
}

if WaypointFound = TRUE
{
	set LZ to WAYPOINT("ALDRIN LZ"):GEOPOSITION.
	set LZSOURCE to "WAYPOINT".
	PRINT "LZ set to WAYPOINT" at (0,4).
}
ELSE IF CheckTarget = TRUE //2nd option, if a landed ship or flag is selected
{
	IF TARGET:Status = "LANDED" OR "SPLASHED"
	{
		set LZ to TARGET:GEOPOSITION.
		set LZSOURCE to "TARGET".
		PRINT "LZ set to TARGET" at (0,4).
	}
}
ELSE //if no other targets, use ships position in 90 degrees
{
	set LZ to circle_destination(ship:geoposition,vel_bearing(),SHIP:BODY:RADIUS * constant:pi * 0.5). //trace around the surface by the current surface velocity heading for 1/4 the body's circumference
	if lz:lng < -180
	{
		set NEWLNG to lz:lng + 360. //if longitude was nudged to an improper value, bring it back round to a proper one
		set NEWLZ to LATLNG(LZ:LAT,NEWLNG).
		set LZ to NEWLZ.
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
	set OrbitAligned to FALSE.
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
		SET KUNIVERSE:TIMEWARP:WARP to 2. //100x timewarp to adjustment node
		until SHIP:GEOPOSITION:LNG < NodeLNG + 0.5 AND SHIP:GEOPOSITION:LNG > NodeLNG - 0.5 //until ship is within 0.5 degrees of adjustment node longitude
		{
			if SHIP:GEOPOSITION:LNG < NodeLNG + 5 AND SHIP:GEOPOSITION:LNG > NodeLNG - 5
			{
				SET KUNIVERSE:TIMEWARP:WARP to 1.
			}
			wait 0. //do nothing
		}
		Set Warp to 0.
		StatusText("Adjusting Course").
		//ship is now at orbital adjustment node
		
		//adjust heading to correct course
		
		Lock CrsError to vel_bearing()-circle_bearing(ship:geoposition,LZ).
		SET northPole TO latlng(90,0).
		LOCK ShipHeading TO mod(360 - northPole:bearing,360).
		
		
		if CrsError > 0 //ship's course bearing is greater than required,
		{
			set HDGPoint to vel_bearing()-90.
			if HDGPoint < 0
			{
				set HDGPoint to HDGPoint + 360.
			}
		}
		else
		{
			set HDGPoint to vel_bearing()+90.
			if HDGPoint > 360
			{
				set HDGPoint to HDGPoint - 360.
			}
		}
		lock steering to heading(HDGPoint,0).
		
		set Orient to FALSE.
		until Orient = TRUE //until we are pointing in the right direction
		{
			if abs(ShipHeading - HDGPoint) < 0.1
			{
				wait 1. //wait 1 second before checking again
				if abs(ShipHeading - HDGPoint) < 0.1 //if still oriented correctly (i.e. heading has settled)
				{
					set Orient to TRUE.
				}
			}
			wait 0.
		}
		wait TurnTime. //wait 2 seconds for ship to settle
		
		if ULLAGE = TRUE //if ullage is needed
		{
			Set Ship:Control:FORE to 1.
			Wait 1.
		}
		
		set throttle to 1. //ignite engines to correct course
		Set Ship:Control:Fore to 0.
		
		until abs(CrsError) < AlignmentTolerance
		{
			wait 0.
		}
		
		set throttle to 0. //cut engines
		unlock steering.
		SET OrbitAligned TO TRUE.
		StatusText("Orbital Plane Aligned").
	}
	
	Set GateAlt to LZ:TERRAINHEIGHT + GateHeight. //calculate the altitude of the approach gate
	StatusText("Generating Plan").
	Set Plan to FlightPlan(MinThrottle,GateAlt,ShipThrust,ship:orbit:periapsis,0,ship:groundspeed,ship:mass,ShipISP). //run the function to create a flight plan for the descent burn, not using totaly accurate starting data yet.
	if Plan[0][0] < 0 //if the first distance in the flightplan is negative (done in case of error in the plan function)
	{
		//do nothing further
		print "DEBUG: Plan Reports Error".
	}
	else //otherwise continue
	{
		StatusText("Plan Generated").
		lock LZRange to circle_distance(Ship:Geoposition,LZ).
		
		//warp to near braking burn start
		SET KUNIVERSE:TIMEWARP:WARP to 2. //100x timewarp to braking burn start
		until circle_distance(Ship:geoposition,LZ) < abs(Plan[0][0])+(ship:groundspeed*10*TurnTime) //until ship is within the distance travelled in 100 turn times of the braking start
		{
			print "DIST " at (0,9).  print round((LZRange/1000),2) at (6,9). print " " + Round(abs(Plan[0][0]/1000),2) at (16,9). Print "KM" at (34,9).
			if circle_distance(Ship:geoposition,LZ) < abs(Plan[0][0])+(ship:groundspeed*30*TurnTime)
			{
				SET KUNIVERSE:TIMEWARP:WARP to 1.
			}
			wait 0.
		}
		SET KUNIVERSE:TIMEWARP:WARP to 0.
		
		//set up display for descent information
		print "      CURRENT    PLANNED" 			at (0,8). //header line
		print "DIST " 	at (0,9).
		print "HSPD "	at (0,10).
		print "VSPD "	at (0,11).
		print "ALT  "	at (0,12).
		print "CRS  "	at (0,13).
		print "HCOR "	at (0,14).
		print "PIT  "	at (0,15).
		
		lock steering to SRFRETROGRADE. //lock the steering to retrograde
		
		//recalc flightplan with accurate starting data
		StatusText("Regenerating Plan").
		Set Plan to FlightPlan(MinThrottle,GateAlt,ShipThrust,ship:altitude,ship:verticalspeed,ship:groundspeed,ship:mass,ShipISP).
		StatusText("Plan Regenerated").
		
		set RUNMODE to "BRAKE".
	}
}

//runmode brake
//engage engines for descent, burn to follow flight plan
	//start by braking at 0* pitch, heading adjustments by simply offsets
	//once desired vertical speed is achieved, set default control inputs to maintain vertical speed (HDG remains unchanged)
	//simple offsets in place to correct for hi/lo altitude and speed for set distance to target
	//aim for "Gate" at 1km before and 1km above landing target
	
if RUNMODE = "BRAKE"
{
	until circle_distance(Ship:geoposition,LZ) <= abs(Plan[0][0]) //wait until distance to LZ is equal to descent plan distance
	{
		print "DIST " at (0,9).  print round((LZRange/1000),2) at (6,9). print " " + Round(abs(Plan[0][0]/1000),2) at (16,9). Print "KM" at (34,9).
		wait 0.
	}
	
	if ULLAGE = TRUE
	{
		set Ship:Control:Fore to 1.
		Wait 1.
	}
	
	set BurnStartMass to ship:mass. //remember the ship mass at burn start, to be used to calculate used DV later
	
	StatusText("Braking Burn").
	set PlanThrot to 0.95.
	set ThrotAdjust to 0.
	lock throttle to max(0.001,min(1,PlanThrot + min(0.05,max(-0.7,ThrotAdjust)))). //throttle lock calculation, throttle setting bracketed between 0.001 and 1 to prevent engine shutdown
	Set Ship:Control:fore to 0.
	
	SOUND:PLAY(DoubleBeep). //play a double beep to alert to starting the descent burn
	
	lock PlanHDG to Circle_bearing(Ship:geoposition,LZ)-180. //lock default heading to point directly away from LZ on the compass
	set HDGAdjust to 0.
	
	set PlanPitch to 0.
	Set PitchAdjust to 0.
	
	lock Gravity to (ship:body:mu * (ship:Mass*1000)) / (ship:altitude + Ship:Body:radius)^2. //gravity force acting on the ship
	lock LiftNeeded to Gravity - (OrbitLift(ship:groundspeed,ship:altitude) * (Ship:Mass*1000)). //calculate the force needed by the engines to maintain vertical speed
	
	lock steering to heading(PlanHDG + MIN(MaxHdgCorr,MAX(-MaxHdgCorr,HDGAdjust)),max(0,min(88,PlanPitch+MIN(MaxPitchCorr,MAX(-MaxPitchCorr,PitchAdjust))))). //pitch brakceted between 0 and 88
	
	set Entry to 0.
	set GATE to FALSE. //flag to kick out of controlled descent
	set CONTROLLED to FALSE. //Flag to kick into controlled descent
	Set Beep to False.
	
	until GATE = TRUE //loop runs for the duration of the braking burn
	{	
		wait 0. //dont run this loop more than once per tick
		if LZRange < abs(Plan[Entry][0]) //if the next entry in the flight plan is reached
		{
			set Entry to Entry+1. //move to next entry
		}
		
		//print all relevant data points to output
		
		print Clearline at (0,9).
		print "DIST " at (0,9).  print round((LZRange/1000),2) at (6,9). print " " + Round(abs(Plan[Entry][0]/1000),2) at (16,9). Print "KM" at (34,9).
		print Clearline at (0,10).
		print "HSPD " at (0,10). print round(ship:groundspeed,1) at (6,10). print " " + round(Plan[Entry][2],1) at (16,10). print "M/S" at (33,10).
		print Clearline at (0,11).
		print "VSPD " at (0,11). print round(ship:verticalspeed,1) at (6,11). print " " + round(Plan[Entry][3],1) at (16,11). print "M/S" at (33,11).
		print Clearline at (0,12).
		print "ALT  " at (0,12). print round(ship:altitude/1000,2) at (6,12). print " " + Round (Plan[Entry][1]/1000,2) at (16,12). print "KM" at (34,12).
		print Clearline at (0,13).
		print "CRS  " at (0,13). print round(Vel_bearing(),1) at (6,13). print " " + round(circle_bearing(ship:geoposition,LZ),1) at (16,13). print "DEG" at (33,13).
		print Clearline at (0,14).
		print "HCOR " at (0,14). print round(HDGAdjust,1) at (6,14). print "DEG" at (33,14).
		print Clearline at (0,15).
		print "PIT  " at (0,15). print " " + round(PlanPitch+PitchAdjust,1) at (16,15).
		
		//print Clearline at (0,17).
		//print "LiftRatio: " + round((LiftNeeded/(ThrustOutput(ShipThrust*1000,MinThrottle,throttle))),2) at (0,17).
		//print Clearline at (0,18).
		//print "ALTCHECK: " + round(AltCheck,2) at (0,18).
		//print Clearline at (0,19).
		//print "HspdCheck: " + round(HSpdCheck,2) at (0,19).
		//print Clearline at (0,20).
		//print "VSpdCheck: " + round(VSpdCheck,2) at (0,20).
		
		//print Clearline at (0,22).
		//print "PitchAdj: " + round(PitchAdjust,2) at (0,22).
		//print Clearline at (0,23).
		//print "ThrotAdj: " + round(ThrotAdjust,2) at (0,23).
		
		
		if Plan[Entry]:LENGTH = 5 //if a flag exists at this entry
		{
			if Plan[Entry][4] = "HANDOVER"
			{
				set Controlled to TRUE.
				StatusText("Controlled Descent").
			}
			else if Plan[Entry][4] = "GATE"
			{
				Set GATE to TRUE. //flag to kick out of brake and into approach mode
				StatusText("FINAL APPROACH").
			}
		}
		
		//course error correction
		if ship:Groundspeed > 300
		{
			if vel_bearing() < 180 //for prograde orbit
			{
				set HDGAdjust to (vel_bearing()-circle_Bearing(Ship:geoposition,LZ)) * HDGAdjustMult.
			}
			else //retrograde orbit
			{
				set HDGAdjust to (Circle_bearing(Ship:geoposition,LZ)-vel_bearing()) * HDGAdjustMult.
			}
		}
		else //at low speeds, stop adjusting heading to prevent a spin/veering off course
		{
			set HDGAdjust to 0. //stop correcting heading for course correction
			lock PlanHDG to (vel_bearing() -180). //use retrograde heading, rather than away from target
		}
		
		if Controlled = TRUE //only run the controlled descent portion if passed handover point
		{
			if Beep = FALSE //if no beep for controlled start has happened
			{
				SOUND:PLAY(DoubleBeep). //play a beep
				set Beep to TRUE. //don't beep again
			}
			
			if ship:groundspeed < 50 //if speed gets below 50m/s before completing the flightplan (error in control resulted in decelerating too much)
			{
				BREAK. //break out of the control loop and proceed to approach
			}
			
			set CurThru to ThrustOutput(ShipThrust*1000,MinThrottle,throttle).
			set PlanPitch to arcsin(min(1,max(-1,LiftNeeded/CurThru))). //calculate the pitch angle needed to maintain vertical speed at planned thrust (locked the ARCSIN function to within its legal values)
			
			set TgtVSpd to Plan[Entry][3]. //read the target vertical speed from the flight plan
			if LZRange < LevelOffDist //if we are close to the end of the descent and want to begin reducing vertical speed
			{
				set TgtVSpd to TgtVSpd * (abs(LZRange - LevelOffDist) / LevelOffDist). //gradually reduce target vertical speed proportional to how far within the LevelOffDist we are
			}
			set VSpdCheck to 1 - (Ship:VerticalSpeed/TgtVSpd). //ratio between VSpd Error and target VSpd, negative numbers mean descending too fast, positive numbers mean descending too slow (Or ascending if > 1)
			
			//HSpd feedback
			set HSpdCheck to (100 - ((ship:groundspeed/Plan[entry][2]) * 100))*-1.
			if abs(HSpdCheck) > HSpdTolerance
			{
				if HSpdCheck > 0 //ship is going faster than planned
				{
					set ThrotAdjust to (HSpdCheck/100) * ThrotMult * HSpdMult.
				}
				else //ship is going slower than planned
				{
					set PitchAdjust to 0 - (VSpdCheck * VSpdMult * PitchMult).
					set ThrotAdjust to (HSpdCheck/100) * ThrotMult * HSpdMult.
				}
			}			
			
			//ALT feedback
			set ALTCheck to (100 - ((ship:altitude/Plan[entry][1]) * 100))*-1.
			if abs(ALTCheck) > AltTolerance
			{
				if ALTCheck > 0 //current altitude is too high
				{
					if HSpdCheck > 0 //speed is above planned
					{
						set PitchAdjust to (-AltCheck * PitchMult * 0.5 * AltMult) - (VSpdCheck * VSpdMult * PitchMult).
					}
					else //speed is below planned
					{
						//do nothing
					}
				}
				else //current altitude is too low
				{
					if HSpdCheck > 0 //speed is above planned
					{
						set PitchAdjust to 0 - (VSpdCheck * VSpdMult * PitchMult). //do nothing to pitch, except with VSpd feedback
					}
					else //speed is below planned
					{
						//set ThrotAdjust to (AltCheck/100) * -0.1 * ThrotMult * -AltMult.
						set PitchAdjust to (HSpdCheck * PitchMult * -HSpdMult) - (VSpdCheck * 1.5 * VSpdMult * PitchMult).
					}
				}
			}
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
	StatusText("APPROACH GATE").
	SOUND:PLAY(DoubleBeep). //play a beep
	
	set FINISH to false.
	until FINISH = TRUE
	{
		lock steering to SRFRETROGRADE. //start orienting to surface retrograde, leave throttle as it was set before
		lock throttle to 1. //max throttle to prevent overrunning into too low altitude
		
		lock CurPitch to 90 - vectorangle(ship:up:forevector, ship:facing:forevector).
		lock ActAcc to sin(min(89,CurPitch)) * ((ThrustOutput(ShipThrust*1000,MinThrottle,0.85) - Gravity) / (ship:mass*1000)). //calculate the acceleration achievable at 85% throttle, allowing for gravity
		lock SafeVSpd to SQRT(2 * (max(0.1,Alt:RADAR - 10)) * ActAcc). //calculate the safe vertical speed based on a Hoverslam calculation using current altitude (with 10m safety margin) and acceleration calculated above
		
		Set ThrotPID to pidloop(0.55 , 0.15 , 0.5 , 0.001 , 1). //now PID controller takes over to control vertical speed
		
		until ALT:radar < 100
		{
			//print stats
			print Clearline at (0,9).
			print "DIST " at (0,9).  print round(LZRange,0) at (6,9). Print "M" at (34,9).
			print Clearline at (0,10).
			print "HSPD " at (0,10). print round(ship:groundspeed,1) at (6,10). print "M/S" at (33,10).
			print Clearline at (0,11).
			print "VSPD " at (0,11). print round(ship:verticalspeed,1) at (6,11). print "M/S" at (33,11).
			print Clearline at (0,12).
			print "ALT  " at (0,12). print round(Alt:RADAR,1) at (6,12). print "M" at (34,12).
			print Clearline at (0,13).
			print Clearline at (0,14).
			print Clearline at (0,15).
			print Clearline at (0,16).
			print Clearline at (0,17).
			print Clearline at (0,18).
			print Clearline at (0,19).
			
			set ThrotPID:Setpoint to min(-0.5,-SafeVSpd). //try to maintain a descent rate equal to 2.5% the radar altitude, i,e, 2.5m/s at 100m alt.
			set throttle to ThrotPID:update(time:seconds,ship:verticalspeed). //update throttle using PID loop.
			wait 0.
		}
		
		
		StatusText("Final Descent").
		//lock steering to up.
		
		//wait for landing
		until alt:radar < 1
		{
			//print stats
			print Clearline at (0,9).
			print "DIST " at (0,9).  print round(LZRange,0) at (6,9). Print " M" at (34,9).
			print Clearline at (0,10).
			print "HSPD " at (0,10). print round(ship:groundspeed,1) at (6,10). print " M/S" at (33,10).
			print Clearline at (0,11).
			print "VSPD " at (0,11). print round(ship:verticalspeed,1) at (6,11). print " M/S" at (33,11).
			print Clearline at (0,12).
			print "ALT  " at (0,12). print round(Alt:RADAR,1) at (6,12). print " M" at (34,12).
			print Clearline at (0,13).
			print Clearline at (0,14).
			print Clearline at (0,15).
			print Clearline at (0,16).
			print Clearline at (0,17).
			print Clearline at (0,18).
			print Clearline at (0,19).
			
			set ThrotPID:Setpoint to MIN(-0.5,-SafeVSpd). 
			set throttle to ThrotPID:update(time:seconds,ship:verticalspeed). //update throttle using PID loop.
			
			if ship:verticalspeed > -0.1 //if no longer descending
			{
				StatusText("CLIMBING, MECO").
				lock steering to up. //
				break. //kick out of the loop; likely due to contacting the ground, or else because minimum TWR is too high
			}
			
			if ship:status = "LANDED" //if KoS notices that the ship is landed
			{
				StatusText("TOUCHDOWN, MECO").
				break. //kick out of the loop
			}
			
			wait 0.
		}
		
		//release all controls
		unlock throttle.
		set throttle to 0.
		unlock throttle. //i'm not paranoid about throttle cuts... honest...
		
		unlock steering.
		RCS OFF.
		
		until ship:status = "LANDED"
		{
			wait 0.
		}
		
		wait 10.
		
		//print final stats
		set LandingDistanceError to Circle_Distance(Ship:Geoposition,LZ).
		print "Landing Distance Error (M) is:" at (0,14).
		print round(LandingDistanceError,0) at (0,15).
		
		set DvUsed to ShipISP * constant:g0 * LN(BurnStartMass/ship:mass).
		print "Estimated dV of Descent is:" at (0,17).
		print round(DvUsed,0) at (0,18).
		
		
		//deploy panels etc.
		if ManagePanels = TRUE
		{
			panels on.
			StatusText("Deploying Solar Panels").
		}
		set FINISH to TRUE.
	}
}

set config:ipu to OriginalIPU.