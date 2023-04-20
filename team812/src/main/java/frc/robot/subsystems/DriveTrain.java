/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private final WPI_TalonSRX leftFront, leftBack, rightFront, rightBack;
  // private final WPI_TalonSRX m_leftMotor, m_rightMotor;

  // private final SpeedControllerGroup leftMotors, rightMotors;
  // private final MotorControllerGroup leftMotors, rightMotors;
  // private final Encoder rightEncoder, leftEncoder;
  private final DifferentialDrive driveBase;
  private GyroSubsystem m_gyro;
  private EncoderSubsystem m_encoder;
  private double m_initialOrientation = -1000.0; // Robot orientation on field 0..359.9999 or -1 for undefined (90 = starting orientation)
  private double m_initialYaw = 0.0;
  private boolean m_yawInitialized = false;

  public DriveTrain() {

    leftFront = new WPI_TalonSRX(CANConstants.kLeftMotors[0]);
    leftFront.configFactoryDefault();
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftFront.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);

    leftBack = new WPI_TalonSRX(CANConstants.kLeftMotors[1]);
    leftBack.configFactoryDefault();
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftBack.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);

    leftBack.follow(leftFront);

    // Note that the right motors are inverted in order to turn the same
    // direction as the left motors
    rightFront = new WPI_TalonSRX(CANConstants.kRightMotors[0]);
    rightFront.configFactoryDefault();
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightFront.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
    rightFront.setInverted(true);

    rightBack = new WPI_TalonSRX(CANConstants.kRightMotors[1]);
    rightBack.configFactoryDefault();
    rightBack.setNeutralMode(NeutralMode.Brake);
    rightBack.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
    rightBack.setInverted(true);

    rightBack.follow(rightFront);

    driveBase = new DifferentialDrive(leftFront, rightFront);
    driveBase.setSafetyEnabled(false);

    // For xDrive only
    m_gyro = frc.robot.RobotContainer.m_GyroSubsystem;
    m_encoder = frc.robot.RobotContainer.m_EncoderSubsystem;
  }

  public void preussDrive2022(double throttle, double zRotation) {
    double speed = throttle;
    double turn = zRotation;
    String mode;

    // Left - super low speed
    // Middle - low speed
    // Right - Maximum speed

    if (RobotContainer.m_BlackBox.isSwitchLeft()) {
      speed = speed * DriveTrainConstants.kLowSpeed;
      turn = turn * DriveTrainConstants.kTurnLowSpeed;
      mode = "low";
    } else if (RobotContainer.m_BlackBox.isSwitchCenter()) {
      speed = speed * DriveTrainConstants.kMedSpeed;
      turn = turn * DriveTrainConstants.kTurnMedSpeed;
      mode = "med";
    } else {
      speed = speed * DriveTrainConstants.kHighSpeed;
      turn = turn * DriveTrainConstants.kTurnHighSpeed;
      mode = "high";
    }

    SmartDashboard.putString("Pdrive mode", mode);
    SmartDashboard.putNumber("Pdrive throttle", speed);
    SmartDashboard.putNumber("Pdrive turn", turn);

    driveBase.arcadeDrive(-speed, turn, false);
  }

  public void preussDrive(double throttle, double zRotation) {
    double speed = throttle;
    double turn = zRotation*DriveTrainConstants.kTurnMedSpeed; /// Reduced from full speed 3/21/2023 - dph

    speed = speed * DriveTrainConstants.kHighSpeed;
    turn = turn * DriveTrainConstants.kTurnHighSpeed;

    SmartDashboard.putNumber("Pdrive throttle", speed);
    SmartDashboard.putNumber("Pdrive turn", turn);

    driveBase.arcadeDrive(-speed, turn, true);
  }
  
  public void arcadeDrive(double throttle, double turn) {
    SmartDashboard.putNumber("ArcadeThrottle", -throttle);
    SmartDashboard.putNumber("ArcadeTurn", turn);
    driveBase.arcadeDrive(-throttle, turn, false); // CHECK THE SIGNS of throttle and turn!!!
  }

  public void team4698Drive(double throttle, double turn) {
    // This is totally untested.  Transliterated from python 3/15/2023 - dph
    // The intention is to combine the best of constant curvature and basic arcade drive.
    // Throttle may need to be negated.
    // Calculate semi-constant curvature values
    double left = (((throttle + Math.abs(throttle) * turn) + (throttle + turn)) / 2);
    double right = (((throttle - Math.abs(throttle) * turn) + (throttle - turn)) / 2);

    // Determine maximum output
    double m = Math.max(Math.abs(throttle), Math.abs(turn));

    // Scale if needed
    if (m > 1.0) {
        left /= m;
        right /= m;
    }
    SmartDashboard.putNumber("t4698DriveL", left);
    SmartDashboard.putNumber("t4698DriveR", right);
    driveBase.tankDrive(left, right);
  }

  public void tankDrive(double l, double r) {
    driveBase.tankDrive(l, r);
  }
  // speed = (a*speed^3 + b*speed) * c
  // a = 0.2, b = 1.8, c = 0.05

  // Default arcadeDrive constructor squares the inputs.
  // arcadeDrive(-y, x) is equivalent to arcadeDrive(-y, x, true)
  // in this usage, y (throttle), and x (rotation around the z-axis)
  // are squared within the arcadeDrive() method.

  // Cubic arcadeDrive implementation for 2020 wihch flattens
  // the joystick input response curve by cubing (x^3) the
  // joystick input which ranges from -1.0 to 1.0.
  // Sign is maintained due to math, however, for clarity,
  // Math.copysign() is used. The third parameter to arcadeDrive() is false
  // to prevent the inputs from being squared once again.

  // xDrive - eXperimental drive is an attempt to drive the robot
  // from the perspective of the driver instead of from the perspective of the robot.
  // Input from the joystick is interpreted as the vector for driving.
  // For example if the Joystick is Y = 0.5 and X = 0.5 then then the vector would
  // be pointing NorthEast relative the North being straight ahead on the field
  // (45 degrees counter clockwise from the field X axis).
  // The magnitude* for the joystick input is the throttle*.
  // If the robot speed is very slow or stopped or if direction is approximately reversed,
  // then rotation will be performed first, before x or y displacement (aka throttle).
  // If the vector from the joystick changes by more than some percentage xDrive will
  // consider which direction to turn (clockwise vs counterclockwise) to achieve the desired
  // vector, possibly reversing the robot direction (ie forwards travel vs backwards travel).
  // It is likely that an explcit input to reverse direction or rotate may be required as well
  // (e.g. right 90 degrees or left 90 degrees buttons). 
  public void xDrive(double joystickY, double joystickX) {
    double kRotation = 1.0/90.0;
    double kXDeadZone = 0.02; // Treat joystick X input below 0.02 as 0.0;
    double kYDeadZone = 0.02; // Treat joystick Y input below 0.02 as 0.0;
    int    kAngleTransition = 5; // start accelerations if angle is within _x_ degrees.
    double kSpeedTransition = 3; // continue accelerations if speed is _x_ inches per second or greater.
    double throttle = 0.0;
    double zRotation = 0.0;
    double robotRawOrientation = m_gyro.getAngle();
    int robotOrientation = (int) (robotRawOrientation - m_initialOrientation + 90 + 360.0) % 360; // orientation in integer degrees.
    if (robotOrientation > 180) robotOrientation = robotOrientation - 360;

    double robotSpeed = (m_encoder.getLeftNumberRate() + m_encoder.getRightNumberRate() )/2.0; // inches per second.
    // If we have not captured the initial orientation, capture it now.
    if (m_initialOrientation <= -1000.0) {
      m_initialOrientation = robotRawOrientation; // Capture init
      robotOrientation = 90; // default by convention robot is expected to be pointed toward the center of the field.
    }
    double absJoystickX = Math.abs(joystickX);
    double absJoystickY = Math.abs(joystickY);
    // Implement joystick dead zones.
    if (absJoystickX < kXDeadZone) {
      joystickX = 0.0;
      absJoystickX = 0.0;
    }
    if (absJoystickY < kYDeadZone) {
      joystickY = 0.0;
      absJoystickY = 0.0;
    }
    double signJoyStickX = Math.signum(joystickX);
    double signJoyStickY = Math.signum(joystickY);
    double joystickMagnitude = Math.max(absJoystickX, absJoystickY);
    int joystickOrientation = robotOrientation; // Default point to where the robot is pointing.
    int forwardAngle = 0;
    int reverseAngle = 0;
    String fwdRev = "";
    int path=0;

    // Compute the vector of the joystick position
    if (joystickMagnitude > 0.0) {
      if (joystickX > 0.0) {
        joystickOrientation = (int) (Math.atan(joystickY/joystickX)*360/Math.PI/2.0);
        path=1;
      } else if (joystickX < 0.0) {
        if (joystickY >= 0) {
          joystickOrientation = (int) (Math.atan(joystickY/joystickX)*360/Math.PI/2.0);
          path=2;
        } else {
          joystickOrientation = (int) (Math.atan(joystickY/joystickX)*360/Math.PI/2.0) - 180;
          path = 3;
        }
      } else if (joystickY > 0.0) {
        joystickOrientation = 90;
        path=4;
      } else if (joystickY < 0.0) {
        joystickOrientation = -90;
        path=5;
      } else {
        joystickOrientation = robotOrientation;
        path=6;
      }
      // Compute the relative angle of the robot vs the joystick;
      // Need to try both forward and backward relative angles.
      // convert range from 0..360 to -180..180
      forwardAngle = (joystickOrientation - robotOrientation);
      if (forwardAngle > 180) forwardAngle = 360 - forwardAngle;
      if (forwardAngle <= -180) forwardAngle = 360 + forwardAngle;
      if (forwardAngle > 360) forwardAngle = forwardAngle % 360;

      if (Math.abs(forwardAngle) > 180) {
         boolean bad = true;
      }
      if (forwardAngle > 0)
        reverseAngle = forwardAngle - 180;
      else
        reverseAngle = forwardAngle + 180;
      if (forwardAngle > 180 || forwardAngle < -180 || reverseAngle > 180 || reverseAngle < -180) {
        boolean bad = true;
    }
      //if (reverseAngle > 180) reverseAngle = 360 - reverseAngle;
      if (Math.abs(forwardAngle) <= Math.abs(reverseAngle)) {
        fwdRev = "Forward";
        zRotation = joystickMagnitude * forwardAngle * kRotation;
        if (Math.abs(robotSpeed) > kSpeedTransition || Math.abs(forwardAngle) < kAngleTransition ) {
          throttle = joystickMagnitude;
        } else {
          throttle = 0.0; // rotate first, then accelerate
        }
      } else {
        fwdRev = "Reverse";
        zRotation = joystickMagnitude * reverseAngle * kRotation;
        if (Math.abs(robotSpeed) > kSpeedTransition || Math.abs(reverseAngle) < kAngleTransition ) {
          throttle = -joystickMagnitude;
        } else {
          throttle = 0.0; // rotate first, then accelerate
        }
      }
    } else { 
      // no joystick input
      throttle = 0.0;
      zRotation = 0.0;
      path=99;
    }
    // For debug safety, clamp the values to safe levels
    throttle = MathUtil.clamp(throttle, -0.1, 0.1);
    zRotation = MathUtil.clamp(zRotation, -0.2, 0.2);
    preussDrive(throttle, -zRotation);
    System.out.printf("%d %d %f %f %f %f %d %d %s %d\n"
    , robotOrientation, joystickOrientation, joystickX, joystickY, throttle, zRotation, forwardAngle, reverseAngle, fwdRev, path);

  } // xDrive
  public void setOrientation(int angle) {
    m_initialOrientation = m_gyro.getAngle() - angle + 90;
    if (m_initialOrientation >= 360) m_initialOrientation -= 360;
    if (m_initialOrientation <    0) m_initialOrientation += 360;
  }

  // normalizeYaw - helper function to ensure that yaw angles are normalized to the range of -180 <= yaw < 180
  public double normalizeYaw(double yaw) {
    while (yaw < -180) yaw += 360;
    while (yaw >= 180) yaw -= 360;
    return yaw;
  }

  // joystickToYaw - helper function to convert joystick X/Y to yaw.
  public static double joystickToYaw(double joystickY, double joystickX) {
    double yaw = 0.0; // return 0 if the calculations fall through
    if (joystickX == 0.0) {
      if (joystickY == 0.0) {
        yaw = 0.0; // no joystick input so using 0.0 as a sentinel value.  Perhaps should use robot orientation?
      }
      else if (joystickY < 0) {
        yaw = 0.0; // Straight ahead
      }
      else {
        yaw = -180.0; // Straight behind.
      }
    } else if (joystickX > 0.0) {
        if (joystickY == 0.0) {
          yaw = 90.0;
        } else if (joystickY < 0.0) {
          // vector is somewhere in the first quadrant (ie between North and East)
          yaw = Math.atan(-joystickX/joystickY)*360/Math.PI/2.0;
        } else {
          // vector is somewhere in the fourth quadrant (ie between East and South)
          yaw = 180 - Math.atan(joystickX/joystickY)*360/Math.PI/2.0;
        }
    } else {
      if (joystickY == 0.0) {
        yaw = -90.0;
      } else if (joystickY < 0.0) {
        // vector is somewhere in the first quadrant (ie between North and West)
        yaw = Math.atan(-joystickX/joystickY)*360/Math.PI/2.0;
      } else {
        // vector is somewhere in the fourth quadrant (ie between West and South)
        yaw = -180 - Math.atan(joystickX/joystickY)*360/Math.PI/2.0;
      }
    }
    return yaw;
  }

  // setRobotYaw - Helper function for testing to set the robot yaw to a known value.
  public void setRobotYaw(double yaw) {
    m_yawInitialized = true;
    m_initialYaw = normalizeYaw(-yaw); // robotYaw = normalizeYaw(m_gyro.getAngle() - m_initialYaw);
  }
  
  // yawDrive - an experimental drive is an attempt to drive the robot
  // from the perspective of the driver instead of from the perspective of the robot.
  // Similar to xDrive but using Yaw conventions for coordinates with 
  // Orientation == Yaw == 0 straight ahead and clockwise rotation is positive.
  // In xDrive Orientation = Yaw + 90 and counterclockwise rotation is positive.
  // Input from the joystick is interpreted as the vector for driving.
  // For example if the Joystick is Y = 0.5 and X = 0.5 then then the vector would
  // be pointing NorthEast relative the North being straight ahead on the field
  // (45 degrees counter clockwise from the field X axis).
  // The magnitude* for the joystick input is the throttle*.
  // If the robot speed is very slow or stopped or if direction is approximately reversed,
  // then rotation will be performed first, before x or y displacement (aka throttle).
  // If the vector from the joystick changes by more than some percentage xDrive will
  // consider which direction to turn (clockwise vs counterclockwise) to achieve the desired
  // vector, possibly reversing the robot direction (ie forwards travel vs backwards travel).
  // It is likely that an explcit input to reverse direction or rotate may be required as well
  // (e.g. right 90 degrees or left 90 degrees buttons). 
  public void yawDrive(double joystickY, double joystickX) {
    double kRotation = 1.0/90.0; // Initial guess for P as in PID for the rotation speed vs desired change in yaw.
    double kXDeadZone = 0.02;    // Treat joystick X input below 0.02 as 0.0;
    double kYDeadZone = 0.02;    // Treat joystick Y input below 0.02 as 0.0;
    double kAngleTransition = 5.0; // start throttle accelerations if angle is within _x_ degrees.
    double kSpeedTransition = 3.0; // continue throttle accelerations if speed is _x_ inches per second or greater.
    double throttle = 0.0;
    double zRotation = 0.0;
    double robotYaw = normalizeYaw(m_gyro.getAngle() - m_initialYaw);
    double robotSpeed = (m_encoder.getLeftNumberRate() + m_encoder.getRightNumberRate() )/2.0; // inches per second.
    // If we have not captured the initial yaw from the gyro, capture it now.
    if (!m_yawInitialized) {
      m_yawInitialized = true;
      m_initialYaw = robotYaw;
      robotYaw = 0;
    }
    
    // Implement joystick dead zones.
    if (Math.abs(joystickX) < kXDeadZone) {
      joystickX = 0.0;
    }
    if (Math.abs(joystickY) < kYDeadZone) {
      joystickY = 0.0;
    }
    
    double joystickMagnitude = Math.max(Math.abs(joystickX), Math.abs(joystickY));
    double joystickYaw = robotYaw; // Default point to where the robot is pointing.
    double forwardDeltaYaw = 0.0;
    double reverseDeltaYaw = 0.0;
    String fwdRev = "";

    // Compute the vector of the joystick position
    if (joystickMagnitude > 0.0) {

      // compute the yaw implied by the joystick Y,X
      joystickYaw = joystickToYaw(joystickY, joystickX);
      
      // Compute the relative angle of the robot vs the joystick;
      // Need to try both forward and backward relative angles to see which requires less turning.
      forwardDeltaYaw = normalizeYaw(joystickYaw - robotYaw);
      reverseDeltaYaw = normalizeYaw(forwardDeltaYaw + 180);
      
      if (Math.abs(forwardDeltaYaw) <= Math.abs(reverseDeltaYaw)) {
        fwdRev = "Forward";
        zRotation = joystickMagnitude * forwardDeltaYaw * kRotation;
        if (Math.abs(robotSpeed) > kSpeedTransition || Math.abs(forwardDeltaYaw) < kAngleTransition ) {
          throttle = joystickMagnitude;
        } else {
          throttle = 0.0; // rotate first, then accelerate
        }
      } else {
        fwdRev = "Reverse";
        zRotation = joystickMagnitude * reverseDeltaYaw * kRotation;
        if (Math.abs(robotSpeed) > kSpeedTransition || Math.abs(reverseDeltaYaw) < kAngleTransition ) {
          throttle = -joystickMagnitude;
        } else {
          throttle = 0.0; // rotate first, then accelerate
        }
      }
    } else { 
      // no joystick input
      throttle = 0.0;
      zRotation = 0.0;
    }

    // For debug safety, clamp the values to safe levels
    //throttle = MathUtil.clamp(throttle, -0.1, 0.1);
    //zRotation = MathUtil.clamp(zRotation, -0.2, 0.2);

    preussDrive(throttle, -zRotation);  // TODO Verify sign of zRotation is correct.
    System.out.printf("%f %f %f %f %f %f %f %f %s\n"
    , robotYaw, joystickYaw, joystickX, joystickY, throttle, zRotation, forwardDeltaYaw, reverseDeltaYaw, fwdRev);

  } // yawDrive
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
