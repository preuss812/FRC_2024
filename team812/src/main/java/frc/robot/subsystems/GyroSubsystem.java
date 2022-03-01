/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSubsystem extends SubsystemBase {
  /**
   * Creates a new GyroSubsystem.
   */
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
   
  public GyroSubsystem() {
    gyro.calibrate();
    System.out.println("*** Gyro Subsystem initialized.");
  }
  public double getAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
    SmartDashboard.putNumber("gyroRate", gyro.getRate());
    SmartDashboard.putBoolean("gyroConnected", gyro.isConnected());
  }
}
