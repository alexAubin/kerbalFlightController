import krpc
import time, datetime
import subprocess
import math
import numpy as np

startTime = 0

###############################################################################

def now() : 
    return datetime.datetime.now().replace(microsecond=0)

def say(text) :

    if (startTime != 0) :
        print("["+str(now() - startTime)+"] "+text)
    else :
        print(text)

    subprocess.Popen(["espeak", "-v","en+f5," ,"-a", "70", "-s", "150", "-p", "30" , text])
    
def normalizeAngle(angle) :
    while (angle >  math.pi) : angle = angle - 2 * math.pi
    while (angle < -math.pi) : angle = angle + 2 * math.pi
    return angle

def norm(v) :
    return math.sqrt(v[0]**2+v[1]**2+v[2]**2)

###############################################################################

def countdown(c, sayIt = True) :
    while (c != 0) :
        if (sayIt) : say(str(c))
        c = c-1
        time.sleep(1.0)

def waitAltitude(alt) :
    while vessel.flight().mean_altitude < alt :
        time.sleep(1)

# Vis-viva equation
def deltaV(r, a1, a2, body = "current") :
    if (body == "current") :
        mu = vessel.orbit.body.gravitational_parameter
    else :
        mu = conn.space_center.bodies[body].gravitational_parameter
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    return delta_v

def burnTime(delta_v) :
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate   
    return burn_time

def executeManeuver(node, burn_ut, burn_duration) :

    vessel.control.throttle = 0

    # Warp before start of maneuver
    lead_time = 10
    conn.space_center.warp_to(burn_ut - burn_duration/2. - lead_time)
    
    # Orientate ship
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0,1,0)
    vessel.auto_pilot.wait()
    say('Ready to execute burn')

    # Wait until start of maneuver
    while (ut() < (burn_ut - burn_duration/2)) :
        time.sleep(0.1)

    # Execute burn
    vessel.control.throttle = 1
    time.sleep(burn_duration - 0.5)
    say('Fine tuning')
    vessel.control.throttle = 0.05
    remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
    while remaining_burn()[1] > 0:
        time.sleep(0.1)
    vessel.control.throttle = 0
    node.remove()



def startTransfer(targetBodyName) :
    
    targetBody = conn.space_center.bodies[targetBodyName]
    currentBody = vessel.orbit.body
    currentBodyFrame = currentBody.non_rotating_reference_frame
   
    # Step 1 : compute transition time from current orbit to rendez-vous point
    initialR = (vessel.orbit.apoapsis + vessel.orbit.periapsis) / 2
    transitionSemimajorAxis = (initialR + targetBody.orbit.radius +
            targetBody.equatorial_radius) / 2
    mu = currentBody.gravitational_parameter
    transitionTime = math.pi * math.sqrt(transitionSemimajorAxis**3 / mu) 

    # Step 2 : compute which position will target have on his orbit at
    # rendez-vous
    targetInitialPosition = targetBody.position(currentBodyFrame)
    targetInitialAngle    = math.atan2(targetInitialPosition[2],targetInitialPosition[0])
    targetAngularVelocity = (2 * math.pi) / targetBody.orbit.period 
    targetPredictedAngle  = normalizeAngle(targetInitialAngle + transitionTime * targetAngularVelocity) 

    # Step 3 : compute at which time we must start leaving current orbit
    vesselPosition        = vessel.position(currentBodyFrame)
    vesselAngle           = math.atan2(vesselPosition[2],vesselPosition[0])
    vesselAngularVelocity = (2 * math.pi) / vessel.orbit.period

    startAngleForVessel   = abs(targetPredictedAngle + math.pi + 0.1)
    while (startAngleForVessel <  vesselAngle) :
           startAngleForVessel = startAngleForVessel + 2 * math.pi
    
    angularDiff = startAngleForVessel - vesselAngle
    timeBeforeNode = angularDiff / vesselAngularVelocity
    nodeTime = ut() + timeBeforeNode

    # Step 4 : create node
    delta_v = deltaV(vessel.orbit.apoapsis, initialR, transitionSemimajorAxis)
    node = vessel.control.add_node(nodeTime, prograde=delta_v)
    burn_duration = burnTime(delta_v)

    # Step 5 : execute maneuver
    executeManeuver(node, nodeTime, burn_duration)

def middleTransfer(targetBodyName) :
    
    targetBody = conn.space_center.bodies[targetBodyName]
    next_orbit = vessel.orbit.next_orbit
    time_to_soi_change = vessel.orbit.time_to_soi_change
    
    delta_v = deltaV(next_orbit.periapsis, next_orbit.semi_major_axis, next_orbit.periapsis, body = targetBodyName)
    nodeTime = ut() + time_to_soi_change + next_orbit.time_to_periapsis
    node = vessel.control.add_node(nodeTime, prograde=delta_v)
    burn_duration = burnTime(delta_v)

    executeManeuver(node, nodeTime, burn_duration)
    


###############################################################################

conn = krpc.connect(name='Flight script')
vessel = conn.space_center.active_vessel

# Set up streams for telemetry
ut                = conn.add_stream(getattr, conn.space_center, 'ut')
altitude          = conn.add_stream(getattr, vessel.flight(),   'mean_altitude')
apoapsis          = conn.add_stream(getattr, vessel.orbit,      'apoapsis_altitude')
periapsis         = conn.add_stream(getattr, vessel.orbit,      'periapsis_altitude')
eccentricity      = conn.add_stream(getattr, vessel.orbit,      'eccentricity')

#stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
#stage_3_resources = vessel.resources_in_decouple_stage(stage=3, cumulative=False)
#srb_fuel          = conn.add_stream(stage_3_resources.amount,   'SolidFuel')
#launcher_fuel     = conn.add_stream(stage_2_resources.amount,   'LiquidFuel')

def main() :

    ####################
    #  Initial config  #
    ####################

    vessel.auto_pilot.target_pitch_and_heading(90,90)
    vessel.auto_pilot.engage()
    vessel.control.throttle = 0
    vessel.control.rcs = False
    vessel.control.sas = False
    vessel.auto_pilot.set_pid_parameters(20,5,5)

    #middleTransfer("Mun")
    #exit()

    ####################
    #  Launch          #
    ####################

    countdown(3)
    global startTime
    startTime = now()
    say('Ignition')
    vessel.control.activate_next_stage()
    vessel.control.throttle = 0.5
    time.sleep(2)
    say('Starting gravity turn')
    gravityTurn()
    say('Target apoapsis reached')
    #vessel.control.activate_next_stage()
    #time.sleep(2)
    say('Starting circularization burn')
    circularizationBurn()
    vessel.control.activate_next_stage()
    time.sleep(2)
    say('Starting transfer to Mun')
    startTransfer("Mun")
    middleTransfer("Mun")

###############################################################################

def gravityTurn() :

    turn_start_altitude = 1000
    turn_end_altitude   = 70000
    target_altitude     = 150000

    boostersNotSeparatedYet = True
    turn_angle = 0

    while True :

        # Progressive gravity turn
        if (altitude() > turn_start_altitude) and (altitude() < turn_end_altitude):
            frac = (altitude() - turn_start_altitude) / (turn_end_altitude - turn_start_altitude)
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

        # Separate boosters when no fuel left
        if (boostersNotSeparatedYet) and (vessel.resources.amount('SolidFuel') < 0.1) :
            say('Booster separation')
            vessel.control.activate_next_stage()
            boostersNotSeparatedYet = False
            while vessel.control.throttle < 1.0 :
                  vessel.control.throttle = vessel.control.throttle + 0.1
                  time.sleep(0.1) 

        # If getting close to target altitude, get out of loop
        if (apoapsis() > target_altitude*0.9):
            say('Approaching target apoapsis')
            break

    # Disable engines when target apoapsis is reached
    vessel.control.throttle = 0.25
    while apoapsis() < target_altitude:
        time.sleep(0.1)
    vessel.control.throttle = 0
    time.sleep(1)


def circularizationBurn() :

    delta_v = deltaV(vessel.orbit.apoapsis, vessel.orbit.semi_major_axis, vessel.orbit.apoapsis)
    burn_duration = burnTime(delta_v)
    node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)
    burn_ut = ut() + vessel.orbit.time_to_apoapsis

    executeManeuver(node,burn_ut,burn_duration)

###############################################################################

main()
