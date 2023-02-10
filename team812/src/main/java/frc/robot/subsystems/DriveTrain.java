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
 // private final WPI_VictorSPX leftFront, leftBack, rightFront, rightBack;
 // private final WPI_VictorSPX leftBack, rightFront, rightBack;
  private final WPI_VictorSPX m_leftMotor, m_rightMotor;

 // private final SpeedControllerGroup leftMotors, rightMotors;
// private final MotorControllerGroup leftMotors, rightMotors;
//  private final Encoder rightEncoder, leftEncoder;
  private final DifferentialDrive driveBase;

  public DriveTrain() {
/*
    leftFront = new WPI_VictorSPX(CANConstants.kLeftMotors[0]);
    leftBack = new WPI_VictorSPX(CANConstants.kLeftMotors[1]);
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    leftFront.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
    leftBack.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
    leftMotors = new MotorControllerGroup(leftFront,leftBack);
    */
  m_leftMotor = new WPI_VictorSPX(CANConstants.kLeftMotors[1]);
  m_leftMotor.configFactoryDefault();
  m_leftMotor.setNeutralMode(NeutralMode.Brake);
  m_leftMotor.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
/*
    rightFront = new WPI_VictorSPX(CANConstants.kRightMotors[0]);
    rightBack = new WPI_VictorSPX(CANConstants.kRightMotors[1]);
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
*/
  m_rightMotor = new WPI_VictorSPX(CANConstants.kRightMotors[1]);
  m_rightMotor.configFactoryDefault();
  m_rightMotor.setNeutralMode(NeutralMode.Brake);
  m_rightMotor.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
  m_rightMotor.setInverted(true);


//    driveBase = new DifferentialDrive(leftMotors, rightMotors);
    driveBase = new DifferentialDrive(m_leftMotor, m_rightMotor);
    driveBase.setSafetyEnabled(false);
  }

  public void preussDrive(double throttle, double zRotation) {
  	double speed = throttle;
	  double turn = zRotation;
    String mode;

	// Left - super low speed
	// Middle - low speed
	// Right - Maximum speed
	
	  if( RobotContainer.m_BlackBox.isSwitchLeft() ) {
	    speed = speed * DriveTrainConstants.kLowLowSpeed;
	    turn  = turn  * DriveTrainConstants.kTurnLowLowSpeed;
      mode = "low";
	  } else if( RobotContainer.m_BlackBox.isSwitchCenter() ) {
      speed = speed * DriveTrainConstants.kLowSpeed;
      turn = turn * DriveTrainConstants.kTurnLowSpeed;
      mode = "med";
    } else {
      speed = speed * DriveTrainConstants.kHighSpeed;
      turn  = turn  * DriveTrainConstants.kTurnHighSpeed;
      mode = "high";
	  }

    SmartDashboard.putString("Pdrive mode", mode);
    SmartDashboard.putNumber("Pdrive throttle", throttle);
    SmartDashboard.putNumber("Pdrive turn", turn);

	  driveBase.arcadeDrive(-throttle, turn, false);
  } 
  public void arcadeDrive(double throttle, double turn) {
    driveBase.arcadeDrive(-throttle, turn, false); // CHECK THE SIGNS of throttle and turn!!!
  }
  
  public void tankDrive(double l, double r) {
	  driveBase.tankDrive(l,r);
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
