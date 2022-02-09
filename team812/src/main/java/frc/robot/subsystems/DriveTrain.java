/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveTrainConstants;
import java.lang.Math;
 

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private final WPI_TalonSRX leftFront, leftBack, rightFront, rightBack;
 // private final SpeedControllerGroup leftMotors, rightMotors;
 private final MotorControllerGroup leftMotors, rightMotors;
//  private final Encoder rightEncoder, leftEncoder;
  private final DifferentialDrive driveBase;

  public DriveTrain() {
    leftFront = new WPI_TalonSRX(CANConstants.kLeftMotors[0]);
    leftBack = new WPI_TalonSRX(CANConstants.kLeftMotors[1]);
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
    leftBack.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
   // leftMotors = new SpeedControllerGroup(leftFront, leftBack);
    leftMotors = new MotorControllerGroup(leftFront,leftBack);

    rightFront = new WPI_TalonSRX(CANConstants.kRightMotors[0]);
    rightBack = new WPI_TalonSRX(CANConstants.kRightMotors[1]);
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    rightFront.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
    rightBack.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);

    // New for 2022, motors have to be inverted by this code
    rightFront.setInverted(true);
    rightBack.setInverted(true);

  //  rightMotors = new SpeedControllerGroup(rightFront, rightBack);
   rightMotors = new MotorControllerGroup(rightFront, rightBack);

    driveBase = new DifferentialDrive(leftMotors, rightMotors);
    driveBase.setSafetyEnabled(false);
    
//    driveBase.setRightSideInverted(false);
  }

  // Default arcadeDrive constructor squares the inputs. 
  // arcadeDrive(-y, x) is equivalent to arcadeDrive(-y, x, true)
  // in this usage, y (throttle), and x (rotation around the z-axis)
  // are squared within the arcadeDrive() method.
  public void drive(double throttle, double zRotation) {
  //  driveBase.arcadeDrive(throttle, zRotation);
      driveBase.arcadeDrive(-throttle,zRotation);
  }

  // Cubic arcadeDrive implementation for 2020 wihch flattens
  // the joystick input response curve by cubing (x^3) the
  // joystick input which ranges from -1.0 to 1.0.
  // Sign is maintained due to math, however, for clarity, 
  // Math.copysign() is used. The third parameter to arcadeDrive() is false 
  // to prevent the inputs from being squared once again.
  public void midnightDrive(double throttle, double zRotation) {
    double m_zRotation = Math.pow(zRotation, 3);
    double m_throttle = Math.copySign(throttle,Math.pow(throttle,3));
    driveBase.arcadeDrive(-m_throttle, m_zRotation, false);
  }

  public void doge(double throttle, double zRotation) {

    double m_throttle = throttle;
    double m_zRotation = zRotation;

    // 2021-12 Alex modified the response for X & Y by half
    m_throttle = m_throttle/2.0;
    m_zRotation = m_zRotation/2.0;

    driveBase.arcadeDrive(-m_throttle, m_zRotation, false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
