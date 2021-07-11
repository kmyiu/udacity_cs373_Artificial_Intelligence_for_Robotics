# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
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
    print 'previous error'
    print distance_between((test_target.x, test_target.y), location_estimate)  # error between last estimation and current true robot location
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

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)




