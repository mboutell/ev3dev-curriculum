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
        self.color_sensor = ev3.ColorSensor()
        self.ir_sensor = ev3.InfraredSensor()
        self.beacon_seeker = ev3.BeaconSeeker(channel=1)
        self.pixy = ev3.Sensor(driver_name="pixy-lego")


        # Check that the motors and sensors are actually connected
        assert self.left_motor.connected
        assert self.right_motor.connected
        assert self.arm_motor.connected
        assert self.touch_sensor.connected
        assert self.color_sensor.connected
        assert self.ir_sensor.connected
        assert self.pixy


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

    def stop(self):
        self.stop_left_motor()
        self.stop_right_motor()

    def drive_forward(self, left_speed, right_speed):
        self.drive_left_some(left_speed)
        self.drive_right_some(right_speed)

    def turn_left_until_stop(self, left_speed, right_speed):
        self.left_motor.run_forever(speed_sp=-left_speed)
        self.right_motor.run_forever(speed_sp=right_speed)

    def turn_right_until_stop(self, left_speed, right_speed):
        self.left_motor.run_forever(speed_sp=left_speed)
        self.right_motor.run_forever(speed_sp=-right_speed)


    def loop_forever(self):
        # This is a convenience method that I don't really recommend for most programs other than mqtt/m5.
        #   This method is only useful if the only input to the robot is coming via mqtt.
        #   MQTT messages will still call methods, but no other input or output happens.
        # This method is given here since the concept might be confusing.
        self.running = True
        while self.running:
            time.sleep(0.1)  # Do nothing (except receive MQTT messages) until an MQTT message calls shutdown.

    def shutdown(self):
        # Modify a variable that will allow the loop_forever method to end. Additionally stop motors and set LEDs green.
        # The most important part of this method is given here, but you should add a bit more to stop motors, etc.
        self.running = False

    def seek_beacon(self):
        forward_speed = 300
        turn_speed = 50
        fast_turn_speed = 100

        while not self.touch_sensor.is_pressed:

            current_heading = self.beacon_seeker.heading
            current_distance = self.beacon_seeker.distance
            if current_distance == -128:
                print("IR Remote not found. Distance is -128")
                self.turn_left_until_stop(turn_speed, turn_speed)
            else:
                if math.fabs(current_heading) < 2:
                    print("On the right heading. Distance: ", current_distance, "and heading", current_heading)
                    if current_distance <= 0:
                        self.stop()
                        return True

                    print("On the right heading. Distance: ", current_distance)
                    # You add more!
                    self.drive_forward(forward_speed, forward_speed)
                elif math.fabs(current_heading) < 10:
                    print("Adjusting heading: ", current_heading, "to ", end="")
                    if current_heading < 0:
                        self.turn_left_until_stop(turn_speed, turn_speed)
                        print("left")
                    else:
                        self.turn_right_until_stop(turn_speed, turn_speed)
                        print("right")
                else:
                    print("Heading too far off", current_heading, "so turning ", end="")
                    if current_heading < 0:
                        self.turn_left_until_stop(fast_turn_speed, fast_turn_speed)
                        print("left")
                    else:
                        self.turn_right_until_stop(fast_turn_speed, fast_turn_speed)
                        print("right")

            time.sleep(0.2)

        # The touch_sensor was pressed to abort the attempt if this code runs.
        print("Abandon ship!")
        self.stop()
        return False

