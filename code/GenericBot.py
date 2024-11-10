

from Adafruit_PWM_Servo_Driver import PWM

import time

import RPi.GPIO as GPIO
import re
import os
import math
#import traceback

#import pdb
import logging
loggerMB = logging.getLogger(__name__)
hdlr = logging.FileHandler('MuleBot.log')
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
hdlr.setFormatter(formatter)
loggerMB.addHandler(hdlr)
loggerMB.setLevel(logging.FATAL)

PI = math.pi

class MuleBot:

  """ Class MuleBot
  This class accepts driving commands from the keyboard and it also has
  a target mode where it drives to the target."""

  WHEEL_RADIUS = 2
  # Apparently, in the robot world, the distrance between the two motor
  # driven wheels is called wheel base length.
  WHEEL_BASE_LENGTH = 55
  SECONDS_PER_MINUTE = 60
  MAX_RPM = 12
  RADIANS_IN_CIRCLE = 2.0
  MAX_Radians_PM = RADIANS_IN_CIRCLE * MAX_RPM
  # MAX_RPS = Maximum Radians Per Second
  MAX_RPS = MAX_Radians_PM / SECONDS_PER_MINUTE
  INCHES_PER_METER = 39.3701
  CIRCUM_IN = RADIANS_IN_CIRCLE * math.pi * WHEEL_RADIUS
  CIRCUM_M = CIRCUM_IN / INCHES_PER_METER

  dcMotorPWMDurationLeft = 0
  dcMotorPWMDurationRight = 0


  def __init__(self):
    """
    __init__ initializes class variables.
    """

    global GPIO

    # running is used to control thread execution.
    self._running = True

    # Keep MuleBot parallel to the wall at this distance.
    self.distanceToWall = 0


    self.pwmEnablePin       = 23 # Broadcom pin 23 was 16
    self.motor1DirectionPin = 24 # Broadcom pin 24 was 20
    self.motor2DirectionPin = 25 # Broadcom pin 25 was 21

    self.motorForward = GPIO.HIGH
    self.motorReverse = GPIO.LOW


    self.dcMotorLeftMotor  = 0
    self.dcMotorRightMotor = 1

    self.laserDetectLeftPin  = 6
    self.laserDetectRightPin = 5

    self.motorMaxRPM = 12.0
    self.motorMaxRadiansPM = 2 * self.motorMaxRPM

    # Pin Setup:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
    GPIO.setup(self.pwmEnablePin,       GPIO.OUT)
    GPIO.setup(self.motor1DirectionPin, GPIO.OUT)
    GPIO.setup(self.motor2DirectionPin, GPIO.OUT)

    GPIO.output(self.pwmEnablePin,       GPIO.LOW )

    # This is interupts setups.  They get used with the
    # test() method.
    #GPIO.setup(laserDetectLeftPin,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(laserDetectRightPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    #GPIO.add_event_detect(laserDetectLeftPin,  GPIO.FALLING, callback=myInt)
    #GPIO.add_event_detect(laserDetectRightPin, GPIO.FALLING, callback=myInt)


    # Initialise the PWM device using the default address
    self.pwm = PWM(0x40)
    # Note if you'd like more debug output you can instead run:
    #pwm = PWM(0x40, debug=True)


    #count = 1
    self.pwm.setPWMFreq(1000)                        # Set frequency to 1000 Hz

    self.tgt_min_range = 15


  def terminate(self):
    """terminate is for stopping the thread."""
    self._running = False



  def mps_to_rps(self, mps):
      """
      mps_to_rps transforms meters per second to radians per second.

      @type: float
      @param: mps (meters per second)

      @rtype: float
      @param: rps (radians per second)
      """

      rps = mps * 2 / MuleBot.CIRCUM_M
      return rps

  def rps_to_mps(self, rps):
      """
      rps_to_mps transforms radians per second to meters per second.

      If rps = 2.0, then mps should equal MuleBot.CIRCUM_M because there are
      2.0 radians in a circle.

      @type: float
      @param: rps (radians per second)

      @rtype: float
      @param: mps (meters per second)
      """

      mps = rps * MuleBot.CIRCUM_M / 2
      return mps

  def rps_to_rpm(self, rps):
      """
      rps_to_rpm transforms radians per second to RPM.

      @type: float
      @param: rps (radians per second)

      @rtype: float
      @param: rpm
      """

      rpm = rps * MuleBot.SECONDS_PER_MINUTE / MuleBot.RADIANS_IN_CIRCLE
      return rpm

  def rpm_to_rps(self, rpm):
      """
      rpm_to_rps transforms RPM to radians per second.

      @type: float
      @param: rpm

      @rtype: float
      @param: rps
      """

      rps = rpm / MuleBot.SECONDS_PER_MINUTE * MuleBot.RADIANS_IN_CIRCLE
      return rps

  def rpm_to_mps(self, rpm):
      """
      rpm_to_mps transforms RPM to meters per second.

      @type: float
      @param: rpm

      @rtype: float
      @param: mps
      """

      mps = rpm / 60 * MuleBot.CIRCUM_M
      return mps




  def v(self):
      """v returns the velocity in meters per second."""

      # TODO This translation formula works, but needs simplified.

      # PWM duration can go from 0 to 4095 with 4095 representing max rpm
#      print("MuleBot.v  MuleBot.dcMotorPWMDurationLeft:", MuleBot.dcMotorPWMDurationLeft)
      speed_percentage = float(MuleBot.dcMotorPWMDurationLeft) / 4095.0
#      print("speed_percentage: ", speed_percentage)

      rpm = speed_percentage * self.motorMaxRPM
#      print("rpm: ", rpm)

      secondsPerMinute = 60
      revs_per_second = rpm / secondsPerMinute
#      print("--revs_per_second", revs_per_second)

      inches_per_rev = 2.0 * math.pi * MuleBot.WHEEL_RADIUS
      INCHES_PER_METER = 39.3701
      meters_per_rev =  inches_per_rev / INCHES_PER_METER
#      print("--meters_per_rev", meters_per_rev)

      meters_per_second = meters_per_rev * revs_per_second

#      print("--meters_per_second: ", meters_per_second)
      return meters_per_second




  # I don't think setServoPulse is ever called.
  # Is the pulse parameter ever used?

  #servoMin = 4096 / 12  # Min pulse length out of 4096
  #servoMax = 4095       # Max pulse length out of 4096

  def setServoPulse(channel, pulse):
    """setServoPulse"""

    pulseLength = 1000000                   # 1,000,000 us per second
    pulseLength /= 60                       # 60 Hz
    print ("%d us per period" % pulseLength)
    pulseLength /= 4096                     # 12 bits of resolution
    print ("%d us per bit" % pulseLength)
    pulse *= 1000
    pulse /= pulseLength
    self.pwm.setPWM(channel, 0, pulse)

  def set_wheel_drive_rates(self, v_l, v_r):
      #TODO Update this to handle nevative velocities.

      """ set_wheel_drive_rates set the drive rates of the wheels to the
      specified velocities (rads/s).  The velocities are converted to RPM.


      @type v_l:  float
      @param v_l: velocity left wheel (rads/s)

      @type v_r: float
      @param v_r: velocity right wheel (rads/s)

      """

      # convert to rpm
#      print(">>v_l: ", v_l)
#      print(">>v_r: ", v_r)
      rpm_l = self.rps_to_rpm(v_l)
      rpm_r = self.rps_to_rpm(v_r)
#      print(">>rpm_l: ", rpm_l)
#      print(">>rpm_r: ", rpm_r)

      self.motorSpeed(rpm_l, rpm_r)
      return rpm_l, rpm_r

  def move(self, inches, direction='f'):
      self.stop()

      revolutions = inches / MuleBot.CIRCUM_IN
      rpm = MuleBot.MAX_RPM
      minutes = revolutions / rpm
      seconds = minutes * MuleBot.SECONDS_PER_MINUTE

      v = self.rpm_to_rps(rpm)
      self.motorsDirection(direction)
      self.set_wheel_drive_rates(v, v)

      time.sleep(seconds)
      self.stop()


  def forward(self, inches):
      self.move(inches, 'forward')

  def backward(self, inches):
      self.move(inches, 'backward')

  def stop(self):
        v_l = 0
        v_r = 0
        self.set_wheel_drive_rates(v_l, v_r)

  def turn(self, direction, degrees):
      """
      Performs a turn either to the left or right.
      It should "turn on a dime."

      @param: direction
      @type: string

      @param: degrees
      @type: float
      """

      # Stop the wheels.
      self.stop()

      if direction.lower() in ['l', 'left']:
        direction = 'left'
      elif direction.lower() in ['r', 'right']:
        direction = 'right'
      else:
        print(f"ERROR:  Invalid direction: {direction}")
        import sys
        sys.exit(-1)

      # Calculate radius of turn.  This is correct for turning in-place.
      r_in = MuleBot.WHEEL_BASE_LENGTH / 2

      # They are the same for turning on a dime.
      r_out = r_in

      # Travel distance
      #travel = r_out * PI # This was for a U-turn.
      radians = math.radians(degrees)
      travel = r_out * math.sin(radians) * (PI / 2.0)

      travel_revolutions = travel / MuleBot.CIRCUM_IN

      rpm = MuleBot.MAX_RPM

      #
      # minutes at rpm.
      minutes = travel_revolutions / rpm
      seconds = minutes * MuleBot.SECONDS_PER_MINUTE

      if direction == 'left':
          # M1 forward
          # M2 reverse
          self.motorDirection(self.motor1DirectionPin, self.motorForward)
          self.motorDirection(self.motor2DirectionPin, self.motorReverse)
      else:
          # M1 reverse
          # M2 forward
          self.motorDirection(self.motor1DirectionPin, self.motorReverse)
          self.motorDirection(self.motor2DirectionPin, self.motorForward)


      v_r = self.rpm_to_rps(rpm)
      v_l = self.rpm_to_rps(rpm)

      # Set wheel drive rates.
      self.set_wheel_drive_rates(v_l, v_r)

      # Sleep during the turn.
      time.sleep(seconds)

      # Stop
      self.stop()

  def u_turn(self, direction, diameter_in):
        """u_turn performs an 180 turn either to the 'left' or right based
        on the diameter of the turn in inches.

        @type: string
        @param: direction

        @type: float
        @type: diameter_in
        """

#        pdb.set_trace()
        # Calculate radius of turn for the inside wheel.
        r_in = diameter_in / 2

        # Outside radius is 20 inches from inside radius.
        r_out = r_in + MuleBot.WHEEL_BASE_LENGTH

        # Outside travel distance
        travel = r_out * PI
        travel_revolutions = travel / MuleBot.CIRCUM_IN

        r_ratio = r_out / r_in
        #r_ratio_half = r_ratio / 2

        speed_multiplier = MuleBot.MAX_RPM / r_ratio

        outside_rpm = r_ratio * speed_multiplier
        inside_rpm = speed_multiplier


        #
        # minutes at outside_rpm
        minutes = travel_revolutions / outside_rpm
        seconds = minutes * MuleBot.SECONDS_PER_MINUTE

        # Something isn't quite perfect.
        if direction == 'left':
            if diameter_in < 25:
                seconds -= 1
            else:
                seconds -= 2
        else:
            if diameter_in < 25:
                seconds += 1
            else:
                seconds += 2

        if direction == 'left':
            v_l = self.rpm_to_rps(inside_rpm)
            v_r = self.rpm_to_rps(outside_rpm)
        else:
            v_r = self.rpm_to_rps(inside_rpm)
            v_l = self.rpm_to_rps(outside_rpm)

        #print("2inside:   rpm: ", inside_rpm)
        #print("2outside:   rpm: ", outside_rpm)

        #print("2.1:   v_l: ", v_l)
        #print("2.1:   v_r: ", v_r)

        # Set wheel drive rates.
        self.set_wheel_drive_rates(v_l, v_r)

        # Sleep during the turn.
        time.sleep(seconds)

        # Stop
        self.stop()

        # Move forward 24 inches.
        self.forward(24)

  def u_turn_supervisor(self, command):
        __doc__ = """u_turn_supervisor parses u_turn commands and then
        calls u_turn.

        Examples: 'ul10' makes a left-handed u_turn with a 10" diameter.
                   'ur40' makes a right-handed u_turn with a 40" diameter.

        @type: string
        @param: command"""

        # strip the initial 'u'
        command = command[1:]
        if len(command) > 0:
            if command[0] == 'l':
                direction = 'left'
            else:
                direction = 'right'

        # strip the direction
        command = command[1:]

        if len(command) > 0:
            diameter = int(command)
            self.u_turn(direction, diameter)


  def _uni_to_diff(self, v, omega):

    """
    _uni_to_diff The is a "unicycle model".  It performs a unicycle to
    "differential drive model" mathematical translation.

    NB: The input/output variable are in DIFFERENT units!  This is because
    the units of R are (meters/radian) does the conversion.

    This came from the 'Sobot Rimulator' by Nick McCrea.

    @type v:  float
    @param v: velocity (m/s)

    @type omega: float
    @param omega: angular velocity (rads/s)

    @rtype: float
    @return: v_l velocity left wheel (rads/s)

    @rtype: float
    @return: v_r velocity right wheel (rads/s)
    """

#    print("--MuleBot._uni_to_diff({:.3f}, {:.3f})".format(v, omega))
    loggerMB.debug("--MuleBot._uni_to_diff({:.3f}, {:.3f})".format(v, omega))

    # v = translation velocity (m/s)
    # omega = angular velocity (rad/s)

    # For some reason, it is necessary to multiply the angle by -1.
    # TODO: Probably have to put this back in.
    omega *= -1.0

    inches_per_meter = 39.3701
    circumference_in = 2.0 * math.pi * MuleBot.WHEEL_RADIUS
    circumference_m = circumference_in / inches_per_meter
    radians_per_circumference = 2.0
    # R = roll?(meters/radian)
    R = circumference_m / radians_per_circumference

    # Get info in inches
    Lin = MuleBot.WHEEL_BASE_LENGTH
    # Convert inches to meters
    Lm = Lin / inches_per_meter

    # All measurements are now metric.
    v_l = ( (2.0 * v) - (omega * Lm) ) / (2.0 * R)
    v_r = ( (2.0 * v) + (omega * Lm) ) / (2.0 * R)
    loggerMB.debug("--MuleBot._uni_to_diff v_l, v_r: {:.3f}, {:.3f}".format(v_l, v_r))

    rpm_l = self.rps_to_rpm(v_l)
    rpm_r = self.rps_to_rpm(v_r)
#    print("--MuleBot._uni_to_diff rpm_l, rpm_r: {:.3f}, {:.3f}".format(rpm_l, rpm_r))
    loggerMB.debug("--MuleBot._uni_to_diff rpm_l, rpm_r: {:.3f}, {:.3f}".format(rpm_l, rpm_r))

    return v_l, v_r

  def motorDirection(self, motorPin, direction):
    """
    motorDirection sets the direction of a single motor.

    Keyword arguments:
    motorPin -- Integer representing the direction pin for a specific motor.

    direction -- Single bit representing fowards or backwards.

    Usage:
        self.motorDirection(self.motor1DirectionPin, self.motorReverse)
    """
    #traceback.print_stack()
    #  print "motorPin: ", motorPin
    #  print "direction: ",  direction
    GPIO.output(motorPin, direction)


  def motorsDirection(self, direction):
    """
    motorsDirection sets the direction of both motors to the same direction.

    Keyword arguments:
    direction -- single character
    """

    direction = direction.lower()
    #print(direction)
    if direction in ['r', 'reverse', 'b', 'backward']:
      self.motorDirection(self.motor1DirectionPin, self.motorReverse)
      self.motorDirection(self.motor2DirectionPin, self.motorReverse)
      #print ("Direction reverse")
    else:
      self.motorDirection(self.motor1DirectionPin, self.motorForward)
      self.motorDirection(self.motor2DirectionPin, self.motorForward)
      #print ("Direction forward")

  def dcMotorLeftTurn(self, duration):
    """dcMotorLeftTurn"""

    print ("From dcMotorLeftTurn: ", MuleBot.dcMotorPWMDurationLeft)
    tempPWMDurationLeft = int( MuleBot.dcMotorPWMDurationLeft * 70 / 100 )  # 98
    self.pwm.setPWM(self.dcMotorLeftMotor, 0, tempPWMDurationLeft)

    # Duration of the turn
    time.sleep(duration)

    # Go straight
    self.pwm.setPWM(self.dcMotorLeftMotor, 0, MuleBot.dcMotorPWMDurationLeft)


  def dcMotorRightTurn(self, duration):
    """dcMotorRightTurn"""

    tempPWMDurationRight = int( MuleBot.dcMotorPWMDurationRight * 70 / 100 )
    self.pwm.setPWM(self.dcMotorRightMotor, 0, tempPWMDurationRight)

    # Duration of the turn
    time.sleep(duration)

    # Go straight
    self.pwm.setPWM(self.dcMotorRightMotor, 0, MuleBot.dcMotorPWMDurationRight)

  def constrainSpeed(self, speedRPM):
      """constrainSpeed ensures 0 <= speedRPM <= max.

      @type speedRPM: float
      @param speedRPM: wheel speedRPM (rpm)

      @rtype: float
      @return: constrained wheel speed (rpm)
      """

      if speedRPM > self.motorMaxRPM:
        speedRPM = self.motorMaxRPM

      if speedRPM < 0.0:
        speedRPM = 0.0

#      print ( "motorSpeed RPM adjusted: ", speedRPM )

      return speedRPM

  def motors__Direction(self, speed_l, speed_r):
    """motorDirection sets the direction of the motors based on the sign of
    the speed.

    @type: float
    @param: speed_l

    @type: float
    @param: speed_r

    """

    if speed_l >= 0:
      self.motorDirection(self.motor1DirectionPin, self.motorForward)
    else:
      self.motorDirection(self.motor1DirectionPin, self.motorReverse)

    """
    if speed_r >= 0:
      self.motorDirection(self.motor2DirectionPin, self.motorForward)
    else :
      self.motorDirection(self.motor2DirectionPin, self.motorReverse)
    """


  def motorSpeed(self, speedRPM_l, speedRPM_r):
    """motorSpeed sets the speed of the motors to the supplied rpm.  This has
    been updated to handle negative speeds.

    @type: float
    @param: speedRPM_l (rpm)

    @type: float
    @param: speedRPM_r (rpm)

    """

    #self.motors__Direction(speedRPM_l, speedRPM_r)

    speedRPM_l = abs(speedRPM_l)
    speedRPM_r = abs(speedRPM_r)

    speedRPM_l = self.constrainSpeed(speedRPM_l)
    speedRPM_r = self.constrainSpeed(speedRPM_r)

#   Left motor
    pwmDuration = 4095.0 * speedRPM_l / self.motorMaxRPM
#    print("MuleBot.motorSpeed Duration left float: ", pwmDuration)
    pwmDuration = int( pwmDuration )
#    print("MuleBot.motorSpeed Duration left int: ", pwmDuration)
    startOfPulse = 0
    self.pwm.setPWM(self.dcMotorLeftMotor, startOfPulse, pwmDuration)
    MuleBot.dcMotorPWMDurationLeft = pwmDuration

#   Right motor
    #Adjust for right motor being faster
    pwmDuration = 4095.0 * speedRPM_r / self.motorMaxRPM
    pwmDuration = pwmDuration * 9727 / 10000  # 98.519113 percent
    pwmDuration = int( pwmDuration )
#    print("MuleBot.motorSpeed Duration right int: ", pwmDuration)
    startOfPulse = 0
    self.pwm.setPWM(self.dcMotorRightMotor, startOfPulse, pwmDuration)
    MuleBot.dcMotorPWMDurationRight = pwmDuration


  def init(self):
    """init"""

    junk = 0
    # This is all interupt stuff for calibrating the speed
    # of the wheels.
    #self.interruptLeftCount  = -2
    #self.interruptRightCount = -2
    #self.startTimeLeft  = 0
    #self.startTimeRight = 0
    #self.lastTimeLeft   = 0
    #self.lastTimeRight  = 0



  def run1(self, _q1, _q2,_qWallDistance):

      """run1 is used to navigate the MuleBot to
       a desired distance from the wall.

       This method is a thread.

       _q1 is the current distance to the wall.
       _qWallDistance is used occasionally to establish
       the desire distance.

       _q2 is used to send steering directions to the run2 thread."""


      timeInRightTurn = 0
      timeInLeftTurn = 0

      while self._running:
          #name = threading.currentThread().getName()
          #print "Consumer thread 1:  ", name

          # This method is the only consumer of _qWallDistance.
          # Therefore checking if the queue is empty works.
          # In a multi-consumer environment, check empty()
          # can cause a race condition.
          if _qWallDistance.empty():
              pass
          else:
              self.distanceToWall = _qWallDistance.get()
              _qWallDistance.task_done()






          currentDistance = _q1.get();
          print ("Current distance: ", currentDistance)

          qSize = _q1.qsize()
          if qSize > 1:
            print ( "***** Distance Queue Size: ", qSize, " *****" )

          # Are we navigating?
          navigating = (self.distanceToWall > 0)
          if navigating:
              print ("Desired distance: ", self.distanceToWall)

              accuracy = 0.5
              # Navigate
              if currentDistance < self.distanceToWall - accuracy:
                  print ("Turn right >>>")
                  timeInRightTurn += 1
                  _q2.put('s1')
              elif currentDistance > self.distanceToWall + accuracy:
                  print ("Turn left <<<")
                  timeInLeftTurn += 1
                  _q2.put('p1')
              else:
                  if ( timeInRightTurn > 0 ):
                      for i in range( timeInRightTurn ):
                          _q2.put('p1')
                      # Reset the time
                      timeInRightTurn = 0
                  if ( timeInLeftTurn > 0 ):
                      for i in range( timeInLeftTurn ):
                          _q2.put('s1')
                      # Reset the time
                      timeInLeftTurn = 0
                  print ("On path.")
          # end if

          _q1.task_done()


  def lidarNav_queue_check(self, q_lidar_nav, tgt_range, tgt_width):
          target_range = tgt_range
          target_width = tgt_width

          # The leading 'n' has been stripped of in the run2 thread.
          if not q_lidar_nav.empty():
             command = q_lidar_nav.get()
             command = command.lower()

             first_char = 0
             if command[first_char] == 'r':
               target_range = float( command[1:] )
             if command[first_char] == 'w':
               target_width = float( command[1:] )

          return target_range, target_width





  def lidarNav_should_i_stay_or_should_i_go(self, tgt_range, angle):
      """lidarNav_should_i_stay_or_should_i_go will stay/stop if MuleBot is
      too close to the target.  Otherwise, it will go/continue.

      @type tgt_range: float
      @param : (inches)

      @type angle: float
      @param : (degrees)

      @rtype: float
      @return: target_range (inches)

      @rtype: float
      @return: angle (radians)
      """
      # Stop if we are too close to the target
      if tgt_range < self.tgt_min_range:
          v_l = 0
          v_r = 0
          self.set_wheel_drive_rates(v_l, v_r)

          # setting the range to zero will stop navigating.
          target_range = 0
          angle_rad = None

      else:
          # Use the updated range for the next run.
          target_range = tgt_range

          # Turn based on the angle to target.
          # Positive angles are left.
          # Negative angles are right.

          # Convert from degrees to radians.
          angle_rad = math.radians(angle)

      return target_range, angle_rad



  def velocity_check(self, v_l, v_r):
      """velocity_check slows down the velocities of the two wheels to stay
      between +-MAX_RPS.


      @type: float
      @param: v_l (radians per second)

      @type: float
      @param: v_r (radians per second)

      @rtype: float
      @param: v_l_prime (radians per second)

      @rtype: float
      @param: v_r_prime (radians per second)

      @rtype: float
      @param: turn_duration (seconds)
      """

      if v_l == v_r:
          turn_duration = 0
          return v_l, v_r, turn_duration

      # Assumption: The robot is going forward or is stationary when it is
      #             hunting for the target, i.e., v >= 0.
      # Assumption: It is not known which direction the robot needs to turn.
      # Fact:       The return values of the _uni_to_diff method are symetrical
      #             about v.
      # Conclusion: Therefore it is only necessary to consider the larger of
      #             v_l and v_r to determine the turn duration due to the RPM
      #             limit of the motors.

      vel_max = max(v_l, v_r)
      if vel_max < MuleBot.MAX_RPS:
          turn_duration = 1
          return v_l, v_r, turn_duration




#      pdb.set_trace()
      turn_duration = vel_max / MuleBot.MAX_RPS

      v = self.v()

      # If v_? > v, v - v_? is negative. If you subract a negative, then
      # it becomes an addition.  So, it ends up on the correct side of
      # velocity.
      v_l_prime = v - (v - v_l) / turn_duration
      v_r_prime = v - (v - v_r) / turn_duration

      return v_l_prime, v_r_prime, turn_duration


  def lidarNav_turn(self, angle_rad):
      """lidarNav_turn performs a turn based on an angle.

      @type: float
      @param: angle_rad

      The return values are used for testing.

      @rtype: float
      @param: v_l (radians per second)

      @rtype: float
      @param: v_r (radians per second)

      @rtype: float
      @param: turn_duration (seconds)

      """

#      print("--MuleBot.lidarNav_turn({:.4f}(rad))".format(angle_rad))

#      pdb.set_trace()

      # What is our current velocity (m/s)
      v = self.v()

      rpm = self.rps_to_rpm( self.mps_to_rps(v)  )
#      print("1:   rpm: ", rpm)

      # Navigate per the angle.
      omega = angle_rad





      # v_l and v_r are in radians per second
      v_l, v_r = self._uni_to_diff(v, omega)

      rpm = self.rps_to_rpm(v_l)
#      print("2l:   rpm: ", rpm)
      rpm = self.rps_to_rpm(v_r)
#      print("2r:   rpm: ", rpm)

#      pdb.set_trace()
      v_l, v_r, turn_duration = self.velocity_check(v_l, v_r)

      rpm = self.rps_to_rpm(v_l)
#      print("3l:   rpm: ", rpm)
      rpm = self.rps_to_rpm(v_r)
#      print("3r:   rpm: ", rpm)

      self.set_wheel_drive_rates(v_l, v_r)

      # Sleep during the turn
      time.sleep(turn_duration)

      # Drive straight
      omega = 0 # zero is no turn



      # Something is wrong with _uni_to_diff.
      v_l, v_r = self._uni_to_diff(v, omega)
      # Override v_l and v_r
      v_l = self.mps_to_rps(v)
      v_r = self.mps_to_rps(v)

      rpm = self.rps_to_rpm(v_l)
#      print("4l:   rpm: ", rpm)
      rpm = self.rps_to_rpm(v_r)
#      print("4r:   rpm: ", rpm)
      self.set_wheel_drive_rates(v_l, v_r)

      return v_l, v_r, turn_duration



  def lidarNav(self, _q2, q_lidar_nav, q_water_pump):

      """lidarNav is used to navigate the MuleBot to
      an object.

      This method is a thread.

      _q2 is used to send steering directions to the run2 thread.
      q_lidar_nav receives target range and width information."""

      # Create the RangeBot instance.
      servo_channel = 3
      range_bot = RangeBot(servo_channel)

      UPDATE_PERIOD = .2
      MINIMUM_MANUEVER_RANGE = 28

      target_range = 0
      target_width = 0

      navigating = False
      was_navigating = False

      loggerMB.info('lidarNav before while loop.')
      while self._running:
          v = self.v()
          rpm = self.rps_to_rpm( self.mps_to_rps(v)  )
#          print("---1:   rpm: ", rpm)

          loggerMB.info('lidarNav before lidarNav_queue_check.')
          target_range, target_width = \
              self.lidarNav_queue_check(q_lidar_nav, target_range, target_width)
          loggerMB.info('lidarNav after lidarNav_queue_check.')

          v = self.v()
          rpm = self.rps_to_rpm( self.mps_to_rps(v)  )
#          print("---2:   rpm: ", rpm)

          # Are we navigating?
          navigating = target_range > 0 and target_width > 0
          loggerMB.debug('lidarNav navigating = {}.'.format(navigating))

          if navigating and not was_navigating:
            q_water_pump.put('won')
            was_navigating = True

          if navigating:
#              v = self.v()
#              print("aMuleBot.lidarNav: v (m/s): ", v)
#              print("bMuleBot.lidarNav: target_range: ", target_range)
#              print("cMuleBot.lidarNav: target_width: ", target_width)

              loggerMB.debug('lidarNav before execute_hunt.')
              loggerMB.debug('lidarNav before execute_hunt. range: {}, width: {}'.format(target_range, target_width))
              #angle, tgt_range, hits = \
              #    range_bot.execute_hunt(target_range, target_width)
              loggerMB.debug('lidarNav after execute_hunt.')

#              v = self.v()
#              print("dMuleBot.lidarNav: v (m/s): ", v)
#              print("eMuleBot.lidarNav: angle (deg): ", angle)
#              print("fMuleBot.lidarNav: tgt_range (inches): ", tgt_range)

              if True:

                  loggerMB.debug('lidarNav before lidarNav_should_i_stay_or_should_i_go.')
                  target_range, angle_rad  = \
                      self.lidarNav_should_i_stay_or_should_i_go(tgt_range, angle)
                  loggerMB.debug('lidarNav after lidarNav_should_i_stay_or_should_i_go.')

#                  v = self.v()
#                  print("gMuleBot.lidarNav: v (m/s): ", v)
#                  print("hMuleBot.lidarNav: target_range: ", target_range)
#                  print("iMuleBot.lidarNav: angle_rad: ", angle_rad)
#                  input("Press [Enter] to continue.")

                  if target_range == 0:
                      q_water_pump.put('woff')
                      navigating = False
                      was_navigating = False

                  # Is a turn required?
                  if target_range > 0 and not (angle_rad == 0):

                      # Only make turns if target >  inches away.
                      if target_range > MINIMUM_MANUEVER_RANGE:
                          # A turn is required.



                          loggerMB.debug('lidarNav before lidarNav_turn.')
                          self.lidarNav_turn(angle_rad)

                  # end target range > 0
              # end if navigating

          time.sleep(UPDATE_PERIOD)

      loggerMB.info('lidarNav after while loop.')


  def intFromStr( self, _string, _index ):
      """intFromStr extract an integer from a string."""

      list = re.findall( r'\d+', _string )
      return int( list[_index] )

  def run2(self, _q2, _qWallDistance, q_lidar_nav, q_water_pump):
        """ run2 is a thread
        It is processing commands from the keyboard
        _q2 is a command queue
        _qWallDistance is the ideal distance from the wall
        q_lidar_nav is target range and width pairs"""

        while self._running:
#                name = threading.currentThread().getName()
#                print ("Consumer thread 2:  ", name)
                qCommand = _q2.get();
#                print ("Here is the command... ", qCommand)
#                print


                qSize = _q2.qsize()
                if qSize > 1:
                  print ( "***** Command Queue Size: ", qSize, " *****" )

                # Change the command to lowercase
                qCommand = qCommand.lower()
                cmd = qCommand
                command = cmd[0]

                if command == 'h':
                  pass
                elif command == 'p':
                  index = 0
                  count = self.intFromStr( cmd, index )
                  print ("Left Turn, ", count, " seconds")
                  self.dcMotorLeftTurn (  count  )
                elif command == 's':
                  index = 0
                  count = self.intFromStr( cmd, index )
                  print ("Right Turn, ", count, " seconds")
                  self.dcMotorRightTurn( count  )
                elif command == 't':
                  self.test()
                elif command == 'u':
                  # U-turn command
                  self.u_turn_supervisor(cmd)
                elif command == 'w':
                  q_water_pump.put(cmd)
                # n is for navigating using Lidar.
                elif command == 'n':
                    if len(cmd) >= 3:
                        if cmd[1] == 'r':
                            # get range to target
                            target_range = cmd[2:]
                            target_range = int(target_range)
#                            print("Target range: ", target_range)
                            q_lidar_nav.put( 'r' + str(target_range) )
                        if cmd[1] == 'w':
                            # get width of target
                            target_width = cmd[2:]
                            target_width = int(target_width)
#                            print("Target width: ", target_width)
                            q_lidar_nav.put( 'w' + str(target_width) )

                    # end if



                elif command == 'z':
                  self.setMotorsDirection('f')

                  # Get speeds (m/m)
                  speeds = cmd[1:]
                  comma_index = speeds.find(',')

                  vmm_l = speeds[0: comma_index]
                  vmm_r = speeds[comma_index + 1:]

                  print("vmm_l: ", vmm_l)
                  print("vmm_r: ", vmm_r)

                  # Convert from meters per minute to meters per second
                  v_l = float(vmm_l) / 60.0
                  v_r = float(vmm_r) / 60.0

                  print("v_l: ", v_l)
                  print("v_r: ", v_r)


                  self.set_wheel_drive_rates(v_l, v_r)

                elif command == 'f' or command == 'r':
                  direction = command
                  print(direction)
                  self.setMotorsDirection(direction)

                  index = 0
                  if len( cmd ) == 1:
                    print ("Invalid input: ", command)
                    print ("Remember to enter the speed.")
                    print ("Please try again.")
                    print
                    continue

                  try:
                    speed = float(cmd[1:])
                  except:
                    print ("Invalid speed: ", cmd[1:])
                    print ("Please try again.")
                    continue

                  print("Speed: ", speed)

                  self.motorSpeed(speed, speed)
                elif command == 'd':
                  index = 0
                  inches = self.intFromStr( cmd, index )
                  _qWallDistance.put( inches )
                else:
                  print ("Invalid input: ", command)
                  print ("Please try again.")


                #time.sleep(4)
                _q2.task_done()
        # End while


        self.shutdown()
        time.sleep(2)




  def laserNav( self, _qCommands ):

      """ Name:  laserNav
          Date:  January 2018

          Arguments:  self

          Purpose:  laserNav

      """

      lastCommandChangeTime = None
      lastCommand = None

      while self._running:

          if not (lastCommandChangeTime == None):
              if not (lastCommand == None):
                  # There is a time and command.

                  # Check if at least 30 seconds have passed since
                  # last state change.
                  TIME_TO_WAIT = 30   # Seconds
                  currentTime = time.time() # Seconds
                  sufficientWaitTime = ( (currentTime - lastCommandChangeTime) > TIME_TO_WAIT )
                  if sufficientWaitTime:
                      _qCommands.put(lastCommand)
                      lastCommandChangeTime = currentTime



          files = os.listdir(".")

          for file in files:
              # Looking for files ending in ".loc"
              if file.endswith(".loc"):
                  print ( file )
                  command = "rm " + file
                  os.system( command )

                  # Determine which of the six states that the
                  # Laser Detector is in and steer accordingly.
                  if file == "LO_FL.loc":
                      _qCommands.put("S6")
                      lastCommand = "S6"
                  elif file == "LO_L.loc":
                      _qCommands.put("S3")
                      lastCommand = "S3"
                  elif file == "LO_C.loc":
                      _qCommands.put("S1")
                      lastCommand = "S1"
                  elif file == "RO_C.loc":
                      _qCommands.put("P1")
                      lastCommand = "P1"
                  elif file == "RO_R.loc":
                      _qCommands.put("P3")
                      lastCommand = "P3"
                  elif file == "RO_FR.loc":
                      _qCommands.put("P6")
                      lastCommand = "P6"
                  else:
                      # This should never happen.
                      print( file, " is an invalid state name" )

                  # Ignoring the check for an invalid state, their
                  # there should have been a command issued.
                  lastCommandChangeTime = time.time()

          time.sleep(0.5)



  def setMotorsDirection(self, _direction):
    """setMotorsDirection sets both motors to the same direction. """

    if _direction == 'f' or _direction == 'F':
      self.motorDirection(self.motor1DirectionPin, self.motorForward)
      #self.motorDirection(self.motor2DirectionPin, self.motorForward)
    elif _direction == 'r' or _direction == 'R':
      self.motorDirection(self.motor1DirectionPin, self.motorReverse)
      #self.motorDirection(self.motor2DirectionPin, self.motorReverse)
    else:
      print ("ERROR: setMotorsDirection bad parameter: " + direction)

  def shutdown(self):
    """shutdown """
    count = 0
    self.pwm.setPWM(0, 0, count)
    self.pwm.setPWM(1, 0, count)

    ### How to use the Enable Pin???
    #TODO:  Put this back in.
    GPIO.output(self.pwmEnablePin, GPIO.HIGH)
    GPIO.cleanup()
    print
    print ("Good Bye!")










def myInt(channel):
  global interruptLeftCount
  global interruptRightCount
  global startTimeLeft
  global startTimeRight


  now = time.time()

  if channel == laserDetectLeftPin:
    interruptLeftCount += 1
    elapsedTime = 0.0
#    print channel, interruptLeftCount

    if interruptLeftCount == 1:
      startTimeLeft = now

    if interruptLeftCount > 0:
      elapsedTime = now - startTimeLeft
    print ("Left ", channel, now, interruptLeftCount, elapsedTime)

  if channel == laserDetectRightPin:
    interruptRightCount += 1
    elapsedTime = 0.0
#    print channel, interruptRightCount

    if interruptRightCount == 1:
      startTimeRight = now

    if interruptRightCount > 0:
      elapsedTime = now - startTimeRight
    print ("Right ", channel, now, interruptRightCount, elapsedTime)






def test():
  laserOn = 0
  lastLaserOn = -1

  startTime = time.time()  # This will get overwritten
  finishTime = time.time()
  lastTime = time.time()

  count = 0
  maxEvents = 20
  while count < maxEvents:
    laserOn = GPIO.input(laserDetectLeftPin)
    #print laserOn

    if lastLaserOn == 0:
      if laserOn ==1:
        now = time.time()
        deltaTime = now - lastTime
        if deltaTime > 3.8:
          print (deltaTime)
          lastTime = now
          if deltaTime > 4.3:
            startTime = now
          else:
            count += 1
            finishTime = now

    lastLaserOn = laserOn

    time.sleep(.01)

  totalDeltaTime = finishTime - startTime
  singleDeltaTime = totalDeltaTime / maxEvents
  print (singleDeltaTime, " * ", maxEvents, " = ", totalDeltaTime)
