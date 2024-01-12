// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private CANSparkMax lf_rotate;

  public DriveTrain() {
    lf_rotate = new CANSparkMax(CANConstants.kSwerveLeftFrontRotate,MotorType.kBrushless);
    lf_rotate.set(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
