# ----------
# Part Three
#
# Now you'll actually track down and recover the runaway Traxbot. 
# In this step, your speed will be about twice as fast the runaway bot,
# which means that your bot's distance parameter will be about twice that
# of the runaway. You can move less than this parameter if you'd 
# like to slow down your bot near the end of the chase. 
#
# ----------
# YOUR JOB
#
# Complete the next_move function. This function will give you access to 
# the position and heading of your bot (the hunter); the most recent 
# measurement received from the runaway bot (the target), the max distance
# your bot can move in a given timestep, and another variable, called 
# OTHER, which you can use to keep track of information.
# 
# Your function will return the amount you want your bot to turn, the 
# distance you want your bot to move, and the OTHER variable, with any
# information you want to keep track of.
# 
# ----------
# GRADING
# 
# We will make repeated calls to your next_move function. After
# each call, we will move the hunter bot according to your instructions
# and compare its position to the target bot's true position
# As soon as the hunter is within 0.01 stepsizes of the target,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot. 
#
# As an added challenge, try to get to the target bot as quickly as 
# possible. 

from robot import *
from math import *
from matrix import *
import random

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    (xy_estimate, OTHER) = estimate_next_pos(target_measurement, OTHER)
    distance = distance_between(hunter_position, xy_estimate)
    if distance > max_distance:
        distance = max_distance
    angle = atan2(xy_estimate[1] - hunter_position[1], xy_estimate[0] - hunter_position[0])
    turning = angle - hunter_heading
    turning = angle_trunc(turning)
    return turning, distance, OTHER
    
def estimate_next_pos(measurement, OTHER = None):
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
    # print distance_between(measurement, location_estimate)
    print 'previous estimation error'
    print distance_between((target.x, target.y), location_estimate)  # error between last estimation and current true robot location
    x_estimate = []
    xy_estimate = []
    (x, xy_estimate, P) = kalman_filter(x ,P, OTHER[0])
    # print "x"
    # print x
    OTHER[1].append(x)
    OTHER[2] = P
    return xy_estimate, OTHER 

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
    print "turn"
    print turn
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
        
    xy_estimate = ((H*x).value[0][0], (H*x).value[1][0])
    return x, xy_estimate, P

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
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





