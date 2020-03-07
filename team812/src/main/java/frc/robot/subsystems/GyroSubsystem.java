/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.analog.adis16448.frc.ADIS16448_IMU;

public class GyroSubsystem extends SubsystemBase {
  /**
   * Creates a new GyroSubsystem.
   */
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();
  private boolean announce = false;
   
  public GyroSubsystem() {
    gyro.reset();
  //  gyro.calibrate();
    System.out.println("*** Gyro Subsystem initialized.");

  }
  public double getAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (! announce) {
      System.out.println("*** Gyro periodic called");
      announce = true;
    }
   /* System.out.printf("gyroAngle %f\n", gyro.getAngle());
    System.out.printf("gyroRate %f\n", gyro.getRate());
    System.out.printf("gyroTemp %f\n", gyro.getTemperature());
*/
    SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
    SmartDashboard.putNumber("gyroRate", gyro.getRate());
    SmartDashboard.putNumber("gyroTemp", gyro.getTemperature());
  }

  public void reset() {
    gyro.reset();
  }
}
