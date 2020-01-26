/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.CANConstants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  private final WPI_TalonSRX leftFront, leftBack, rightFront, rightBack;
  private final SpeedControllerGroup leftMotors, rightMotors;
  private final Encoder rightEncoder, leftEncoder;
  private final DifferentialDrive driveBase;

  public DriveTrain() {
    leftFront = new WPI_TalonSRX(CANConstants.kLeftMotors[0]);
    leftBack = new WPI_TalonSRX(CANConstants.kLeftMotors[1]));
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    leftFront.setNeutralMode(NeutralMode.Coast);
    leftBack.setNeutralMode(NeutralMode.Coast);
    leftMotors = new SpeedControllerGroup(leftFront, leftBack);

    rightFront = new WPI_TalonSRX(CANConstants.kRightMotors[0]);
    rightBack = new WPI_TalonSRX(CANConstants.kRightMotors[1]);
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    rightFront.setNeutralMode(NeutralMode.Coast);
    rightBack.setNeutralMode(NeutralMode.Coast);
    rightMotors = new SpeedControllerGroup(rightFront, rightBack);

    driveBase = new DifferentialDrive(leftMotors, rightMotors);
    driveBase.setRightSideInverted(false);
  }

  public void drive(double speed, double angle) {
    driveBase.arcadeDrive(speed, angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
