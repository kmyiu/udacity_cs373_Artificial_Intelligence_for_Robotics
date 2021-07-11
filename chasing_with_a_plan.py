# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot. 
# But this time, your speed will be about the same as the runaway bot. 
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    import copy
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    (x, rho, OTHER) = estimate_state(target_measurement, OTHER)
    F = matrix([[1., 0., 1., 0.], [0., 1., 0., 1.], [0., 0., cos(rho), -sin(rho)], [0., 0., sin(rho), cos(rho)]]) # next state function
    H = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.]]) # measurement function
    # next movement without plan
    plan_nth_location = 1
    has_plan = False
    xy_estimate = ((H*x).value[0][0], (H*x).value[1][0])
    distance = distance_between(hunter_position, xy_estimate)
    if distance <= max_distance:
        has_plan = True
    else:
        distance = max_distance
    angle = atan2(xy_estimate[1] - hunter_position[1], xy_estimate[0] - hunter_position[0])
    turning = angle - hunter_heading
    turning = angle_trunc(turning)
    # with plan
    x2 = copy.deepcopy(x)
    if has_plan == False:
        for i in range(1,50):
            x2 = F * x2
            xy_estimate = ((H*x2).value[0][0], (H*x2).value[1][0])
            distance = distance_between(hunter_position, xy_estimate)
            if ( distance <= max_distance * (i+1) ):
                plan_nth_location = i + 1
                if distance > max_distance:
                    distance = max_distance
                angle = atan2(xy_estimate[1] - hunter_position[1], xy_estimate[0] - hunter_position[0])
                turning = angle - hunter_heading
                turning = angle_trunc(turning)
                break
    print "plan to catch at n = " + str(plan_nth_location)
    return turning, distance, OTHER
    
def estimate_state(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.

    # OTHER[0] = previous measurements
    # OTHER[1] = previous estimations
    # OTHER[2] = P, uncertainty
    if OTHER == None:
        OTHER = [[], [], []]
    OTHER[0].append(measurement)
    # print "measurement"
    # print measurement
    x = matrix([[0.], [0.], [0.], [0.]]) # initial state (location and velocity)
    P = matrix([[1000., 0., 0., 0.], [0., 1000., 0., 0.], [0., 0., 1000., 0.], [0., 0., 0., 1000.]]) # initial uncertainty
    if len(OTHER[1]) > 0:
        x = OTHER[1][-1]
        P = OTHER[2]
    H = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.]]) # measurement function
    location_estimate = ((H*x).value[0][0], (H*x).value[1][0])
    # error between last estimation and current true robot location
    print 'previous estimation error = ' + str(distance_between((target.x, target.y), location_estimate)) 
    x_estimate = []
    xy_estimate = []
    # x, target turning angle estimation, P
    (x, rho, P) = kalman_filter(x ,P, OTHER[0])
    OTHER[1].append(x)
    OTHER[2] = P
    return x, rho, OTHER 

def median(values):
    n = len(values)
    values.sort()
    if n % 2 == 0:
        median1 = values[n//2]
        median2 = values[n//2 - 1]
        median = (median1 + median2)/2
    else:
        median = values[n//2]
    return median
    
def get_weighted_mean(values):
    n = len(values)
    values.sort()
    if n >= 10:
        values = values[3:-3]
    # print(values)
    n = len(values)
    return sum(values) / n
    
    
def kalman_filter(x, P, measurements):
    turn = 0
    turn_median = 0
    if len(measurements) >= 3:
        cnt = 0
        turn = 0
        turn_list = []
        # for i in range(2, len(measurements)):
        for i in range(max(2,len(measurements) - 5),len(measurements)):
            dir1 = atan2(measurements[i][1] - measurements[i-1][1], measurements[i][0] - measurements[i-1][0])
            dir2 = atan2(measurements[i-1][1] - measurements[i-2][1], measurements[i-1][0] - measurements[i-2][0])
            dir_diff = (dir1 - dir2) % (2 * pi)
            if dir_diff > pi:
                dir_diff = dir_diff - 2 * pi
            cnt = cnt + 1
            turn = turn + dir_diff
            turn_list.append(dir_diff)
        turn = turn / cnt
        turn_median = median(turn_list)
    u = matrix([[0.], [0.], [0.], [0.]]) # external motion
    # F = matrix([[1., 0., 1., 0.], [0., 1., 0., 1.], [0., 0., 1., 0], [0., 0., 0., 1.]]) # next state function
    F = matrix([[1., 0., 1., 0.], [0., 1., 0., 1.], [0., 0., cos(turn), -sin(turn)], [0., 0., sin(turn), cos(turn)]]) # next state function
    H = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.]]) # measurement function
    R = matrix([[1., 0.], [0., 1.]]) # measurement uncertainty
    I = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]) # identity matrix

    n = len(measurements) - 1
    # measurement update
    y = matrix([[measurements[n][0]], [measurements[n][1]] ]) - H * x
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + K * y
    P = (I - K * H) * P
    # prediction
    x = F * x + u
    P = F * P * F.transpose()
        
    # xy_estimate = ((H*x).value[0][0], (H*x).value[1][0])
    return x, turn, P

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)





