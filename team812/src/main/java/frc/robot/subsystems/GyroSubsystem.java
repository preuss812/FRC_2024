/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroSubsystem extends SubsystemBase {
  /**
   * Creates a new GyroSubsystem.
   */
    // AHRS is the class for the NavX IMU / Gyro, but we use the variable
    // name "gyro" because it makes more sense.
    private AHRS gyro;
    private double initialPitch;
    private boolean isPitchSet = false;
   
  public GyroSubsystem() {
      try {
	  gyro = new AHRS(SerialPort.Port.kUSB1);
	  gyro.enableLogging(true);
	  this.reset();
      } catch (RuntimeException ex) {
	  DriverStation.reportError("Error instantiating Gyro:  " + ex.getMessage(), true);
      }
      System.out.println("*** Gyro Subsystem initialized.");
  }
  
  public double getAngle() {
    return gyro.getAngle();
  }
  public double getPitch() {
    double delta =  gyro.getPitch() - initialPitch;
    SmartDashboard.putNumber("Current Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Start Pitch", initialPitch);
    SmartDashboard.putNumber("delta Pitch", delta);


    return gyro.getPitch() - initialPitch;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IMU_Connected", gyro.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", gyro.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
    SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());
    if(! gyro.isCalibrating() && ! isPitchSet) {
      initialPitch = gyro.getPitch();
      isPitchSet = true;
    }
  }

  public void reset() {
    gyro.zeroYaw();
    //gyro.
  }
}
