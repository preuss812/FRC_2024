/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
