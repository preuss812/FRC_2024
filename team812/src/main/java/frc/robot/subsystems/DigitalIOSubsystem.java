// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.EncoderConstants;

public class DigitalIOSubsystem extends SubsystemBase {
  /** Creates a new DigitalIOSubsystem. */
  private final DigitalInput p_left_a;
  private final DigitalInput p_left_b;

  public DigitalIOSubsystem() {
    p_left_a = new DigitalInput(EncoderConstants.kLeftDriveEncoder[0]);
    p_left_b = new DigitalInput(EncoderConstants.kLeftDriveEncoder[1]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Encoder Left A", p_left_a.get());
    SmartDashboard.putBoolean("Encoder left B", p_left_b.get());
  }
}
