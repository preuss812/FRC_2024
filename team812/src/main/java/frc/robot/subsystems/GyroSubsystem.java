/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class GyroSubsystem extends SubsystemBase {
  /**
   * Creates a new GyroSubsystem.
   */
//  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();
   
  public GyroSubsystem() {
//    gyro.calibrate();
    System.out.println("*** Gyro Subsystem initialized.");
  }
  public double getAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("gryoAngleRate", "(A,r) = (" + gyro.getAngle() + "," + gyro.getRate() + ")" );
    SmartDashboard.putString("gyroAccel", "(X,Y,Z) = "    + gyro.getAccelX() + "," +    gyro.getAccelY() + "," +    gyro.getAccelZ() + ")");
    SmartDashboard.putString("gyroRate", "(X,Y,Z) = "     + gyro.getGyroRateX() + "," + gyro.getGyroRateY() + "," + gyro.getGyroRateZ() + ")");
  }
}
