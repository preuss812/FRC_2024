// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.CANConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.DriveTrainConstants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class ArmSubsystem extends SubsystemBase {
  private final WPI_TalonSRX m_arm = new WPI_TalonSRX(CANConstants.kArmMotor);


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_arm.configFactoryDefault();
    m_arm.setNeutralMode(NeutralMode.Brake);
    m_arm.configOpenloopRamp(DriveTrainConstants.kOpenLoopRampRate);
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_arm.setSelectedSensorPosition(0.0);
    m_arm.setInverted(false);
    m_arm.setSensorPhase(false); //Attempts to make it positive
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
