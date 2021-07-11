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
from matrix import *  # Check the matrix.py tab to see how this works.
import random


# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
# def estimate_next_pos(measurement, OTHER = None):
#     """Estimate the next (x, y) position of the wandering Traxbot
#     based on noisy (x, y) measurements."""

#     # You must return xy_estimate (x, y), and OTHER (even if it is None)
#     # in this order for grading purposes.
#     xy_estimate = (3.2, 9.1)
#     return xy_estimate, OTHER


## Start of My code
def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    from copy import deepcopy
    if (OTHER == None):
        OTHER = []
    OTHER.append(measurement)
    OTHER_backup = deepcopy(OTHER)
    xy_estimate = [0, 0]
    if (len(OTHER) <= 1):
        pass
    elif (len(OTHER) == 2):
        xy_estimate[0] = (OTHER[-1][0] * 2 - OTHER[-2][0])
        xy_estimate[1] = (OTHER[-1][1] * 2 - OTHER[-2][1])
    else:
        # rand our data set
        interval_max = len(OTHER_backup) / 6
        interval_max = max(1, min(10, interval_max))
        interval = random.randint(1, interval_max)
        idx = range(interval, len(OTHER_backup) + 1, interval)
        if len(idx) > 3:
            idx = idx[0:3]
        for i in range(len(idx)):
            idx[i] = - idx[i]
        # print(idx)
        idx = idx[::-1]
        OTHER = []
        for j in idx:
            OTHER.append(OTHER_backup[j])
        # if(len(OTHER) > 4):
        #     OTHER = OTHER[-4:]
        # print('TEST')
        # print(interval)
        # print(OTHER_backup)
        # print(OTHER)

        # dist_avg is average distance between two consecutive nodes
        # D = 2 * R * sin(angle / 2)
        dist_avg = 0
        for i in range(1, len(OTHER)):
            dist_avg += distance_between(OTHER[i - 1], OTHER[i])
        dist_avg /= (len(OTHER) - 1)
        # dist2_avg is average distance between node[i] and node[i+2]
        # D' = 2 * R * sin( 2 * angle / 2) = 4 * R * sin(angle / 2) * cos(angle / 2)
        # D' / D = 2 * cos(angle / 2)
        dist2_avg = 0
        for i in range(2, len(OTHER)):
            dist2_avg += distance_between(OTHER[i - 2], OTHER[i])
        dist2_avg /= (len(OTHER) - 2)
        ratio = dist2_avg / dist_avg
        # prevent domain error
        if ratio > 1.999:
            ratio = 1.999
        if ratio < -1.999:
            ratio = -1.999
        angle = acos(ratio / 2) * 2
        radius = dist_avg / 2 / sin(angle / 2)
        print('angle = ' + str(angle) + ',radius = ' + str(radius))
        # collections of potential interior/exterior centers of circle formed by two consecutive nodes
        # the set with correct orientation will contain small error
        centers1 = []
        centers2 = []
        for i in range(1, len(OTHER)):
            dir = [OTHER[i][0] - OTHER[i - 1][0], OTHER[i][1] - OTHER[i - 1][1]]
            dir_perpendicular = [-dir[1], dir[0]]
            dir_perpendicular_norm = distance_between([0, 0], dir_perpendicular)
            try:
                height = sqrt(radius ** 2 - (distance_between(OTHER[i], OTHER[i - 1]) / 2) ** 2)
            except:
                height = 0
            center1 = mid_point(OTHER[i - 1], OTHER[i])
            center1[0] += dir_perpendicular[0] / dir_perpendicular_norm * height
            center1[1] += dir_perpendicular[1] / dir_perpendicular_norm * height
            center2 = mid_point(OTHER[i - 1], OTHER[i])
            center2[0] -= dir_perpendicular[0] / dir_perpendicular_norm * height
            center2[1] -= dir_perpendicular[1] / dir_perpendicular_norm * height
            centers1.append(center1)
            centers2.append(center2)
        # compute the correct set of centers
        centers1_error = err_to_center(centers1)
        centers2_error = err_to_center(centers2)
        # print('centers1_error = ' + str(centers1_error) + ', centers2_error = ' + str(centers2_error))
        real_center = mean_points(centers1)
        if centers1_error > centers2_error:
            real_center = mean_points(centers2)
        # find the angle from center
        # angles is result after % 2 * pi
        angles = []
        for i in range(len(OTHER)):
            angles.append(atan2(OTHER[i][1] - real_center[1], OTHER[i][0] - real_center[0]))
        angles_mean = sum(angles) / len(angles)
        angles_diff = []
        for i in range(1, len(OTHER)):
            diff = angles[i] - angles[i - 1]
            # assume each turn is less than < pi / 2
            if diff % (2 * pi) >= pi:
                diff = diff % (2 * pi) - 2 * pi
            angles_diff.append(diff)
        angles_diff_mean = sum(angles_diff) / len(angles_diff)
        # check if angle is in correct orientation or not
        if (angles_diff_mean * angle < 0):
            angle = - angle
        # get angles result without % 2 * pi
        angles_guess = []
        angles_guess.append(angles[0])
        for i in range(1, len(OTHER)):
            angle_guess = angles[i]
            angles_guess_mean = sum(angles_guess) / len(angles_guess)
            angles_guess_start = angles_guess_mean - (len(angles_guess) / 2.0) * angle
            angles_guess_i = angles_guess_start + i * angle
            while abs(angles_guess_i - angle_guess) > pi:
                if (angles_guess_i - angle_guess) > pi:
                    angle_guess += 2 * pi
                else:
                    angle_guess -= 2 * pi
            angles_guess.append(angle_guess)
        # print('* angles with % 2 pi')
        # print(angles)
        # print('* angles without % 2 pi')
        # print(angles_guess)
        angles_guess_mean = sum(angles_guess) / len(angles_guess)
        angle_prediction = angles_guess_mean + (len(angles_guess) / 2.0 + 0.5) * angle
        xy_estimate[0] = real_center[0] + radius * cos(angle_prediction)
        xy_estimate[1] = real_center[1] + radius * sin(angle_prediction)
        # print('real_center = ' + str(real_center))
        # print('angle_prediction = ' + str(angle_prediction))

        for i in range(len(OTHER)):
            xx = real_center[0] + radius * cos(angles[i])
            yy = real_center[1] + radius * sin(angles[i])
            # print('[measurement] = ' + str(OTHER[i][0]) + ', ' + str(OTHER[i][1]) + ' [calculated x,y] = ' + str(xx) +',' + str(yy))
    print('xy_estimate = ' + str(xy_estimate))

    return xy_estimate, OTHER_backup


def err_to_center(centers):
    guess = [0, 0]
    for i in range(len(centers)):
        for j in range(len(centers[i])):
            guess[j] += centers[i][j]
    guess[0] /= len(centers)
    guess[1] /= len(centers)
    error = 0
    for i in range(len(centers)):
        error += distance_between(guess, centers[i])
    return error


def mid_point(point1, point2):
    pt = [(point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2]
    return pt


def mean_points(points):
    mean = [0, 0]
    for i in range(len(points)):
        mean[0] += points[i][0]
        mean[1] += points[i][1]
    mean[0] /= len(points)
    mean[1] /= len(points)
    return mean


## END of my code

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER=None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    limit = 100
    while not localized and ctr <= limit:
        # while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        print('Error = ' + str(error))
        if error <= distance_tolerance:
            print
            "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == limit:
            print
            "Sorry, it took you too many steps to localize the target."
    return localized


# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER=None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that
    position, so it always guesses that the first position will be the next."""
    if not OTHER:  # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER


# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2 * pi / 34.0, 1.5)
# test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

# demo_grading(naive_next_pos, test_target)
demo_grading(estimate_next_pos, test_target)



