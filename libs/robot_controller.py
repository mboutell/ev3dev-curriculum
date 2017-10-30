"""
  Library of EV3 robot functions that are useful in many different applications. For example things
  like arm_up, arm_down, driving around, or doing things with the Pixy camera.

  Add commands as needed to support the features you'd like to implement.  For organizational
  purposes try to only write methods into this library that are NOT specific to one tasks, but
  rather methods that would be useful regardless of the activity.  For example, don't make
  a connection to the remote control that sends the arm up if the ir remote control up button
  is pressed.  That's a specific input --> output task.  Maybe some other task would want to use
  the IR remote up button for something different.  Instead just make a method called arm_up that
  could be called.  That way it's a generic action that could be used in any task.
"""

import ev3dev.ev3 as ev3
import math
import time

class Snatch3r(object):
    """Commands for the Snatch3r robot that might be useful in many different programs."""

    MAX_SPEED = 900

    def __init__(self):
        self.left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        self.right_motor = ev3.LargeMotor(ev3.OUTPUT_C)
        self.arm_motor = ev3.MediumMotor(ev3.OUTPUT_A)

        self.touch_sensor = ev3.TouchSensor()

        # Check that the motors and sensors are actually connected
        assert self.left_motor.connected
        assert self.right_motor.connected
        assert self.arm_motor.connected
        assert self.touch_sensor


    def drive_inches(self, inches_target, speed_deg_per_second):
        assert self.left_motor.connected
        assert self.right_motor.connected

        degrees_per_inch = 90
        motor_turns_needed_in_degrees = inches_target * degrees_per_inch

        self.left_motor.run_to_rel_pos(
            position_sp=motor_turns_needed_in_degrees,
            speed_sp=speed_deg_per_second,
            stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        self.right_motor.run_to_rel_pos(
            position_sp=motor_turns_needed_in_degrees,
            speed_sp=speed_deg_per_second,
            stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        self.left_motor.wait_while(ev3.Motor.STATE_RUNNING)
        self.right_motor.wait_while(ev3.Motor.STATE_RUNNING)

    def turn_degrees(self, degrees_to_turn, speed_deg_per_second):
        # If degrees_to_turn is negative, then motor_turns will be too, and the formulas will work.
        motor_turns_per_degree_to_turn = 5
        motor_turns_needed = degrees_to_turn * motor_turns_per_degree_to_turn

        self.left_motor.run_to_rel_pos(
            position_sp=-motor_turns_needed,
            speed_sp=speed_deg_per_second,
            stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        self.right_motor.run_to_rel_pos(
            position_sp=motor_turns_needed,
            speed_sp=speed_deg_per_second,
            stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        self.left_motor.wait_while(ev3.Motor.STATE_RUNNING)
        self.right_motor.wait_while(ev3.Motor.STATE_RUNNING)

    def arm_calibration(self):
        """
        Runs the arm up until the touch sensor is hit then back to the bottom again, beeping at both locations.
        Once back at in the bottom position, gripper open, set the absolute encoder position to 0.  You are calibrated!
        The Snatch3r arm needs to move 14.2 revolutions to travel from the touch sensor to the open position.
        """
        self.arm_motor.run_forever(speed_sp=Snatch3r.MAX_SPEED)  # max
        while not self.touch_sensor.is_pressed:  # is_pressed
            time.sleep(0.01)
        self.arm_motor.stop(stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        ev3.Sound.beep().wait()  # missing
        arm_revolutions_for_full_range = 14.2 * 360  # need the 360
        self.arm_motor.run_to_rel_pos(position_sp=-arm_revolutions_for_full_range)
        self.arm_motor.wait_while(ev3.Motor.STATE_RUNNING)  # not stalled
        self.arm_motor.position = 0  # Calibrate the down position as 0 (this line is correct as is).

    def arm_up(self):
        """        Runs the arm up until the touch sensor is hit, then beeps. """
        self.arm_motor.run_forever(speed_sp=Snatch3r.MAX_SPEED)
        while not self.touch_sensor.is_pressed:
            time.sleep(0.01)
        self.arm_motor.stop(stop_action=ev3.Motor.STOP_ACTION_BRAKE)
        ev3.Sound.beep().wait()

    def arm_down(self):
        """Runs the arm to the bottom position, then beeps
        Once in the bottom position, gripper open, sets the absolute encoder position to 0.
        """
        arm_revolutions_for_full_range = 14.2 * 360
        self.arm_motor.run_to_rel_pos(position_sp=-arm_revolutions_for_full_range)
        self.arm_motor.wait_while(ev3.Motor.STATE_RUNNING)
        self.arm_motor.position = 0
        ev3.Sound.beep().wait()

    def drive_left_some(self, speed_deg_per_second):
        self.left_motor.run_forever(speed_sp=speed_deg_per_second)

    def stop_left_motor(self):
        self.left_motor.stop(stop_action=ev3.Motor.STOP_ACTION_COAST)

    def drive_right_some(self, speed_deg_per_second):
        self.right_motor.run_forever(speed_sp=speed_deg_per_second)

    def stop_right_motor(self):
        self.right_motor.stop(stop_action=ev3.Motor.STOP_ACTION_COAST)

    def shutdown(self):
        # What to do here?
        pass