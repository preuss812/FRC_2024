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
    private static double m_xDisplacement = 0.0;
    private static double m_yDisplacement = 0.0;
    private static double m_xVelocity = 0.0;
    private static double m_yVelocity = 0.0;
    private static double m_xAcceleration = 0.0; // In case we decide to do trapezoidal acceleration.
    private static double m_yAcceleration = 0.0; // In case we decide to do trapezoidal acceleration.
    private static final double deltaTime = 0.020; // 20 milliseconds.  Assuming that the ideal time is achieved.

  public GyroSubsystem() {
      try {
	  gyro = new AHRS(SerialPort.Port.kUSB1);
	  gyro.enableLogging(true);
      } catch (RuntimeException ex) {
	  DriverStation.reportError("Error instantiating Gyro:  " + ex.getMessage(), true);
      }
      System.out.println("*** Gyro Subsystem initialized.");
  }
  
  public double getAngle() {
    return gyro.getAngle();
  }
  public double getPitch() {
      // The NavX does not provide a method zeroPitch() so in the
      // periodic function we save the the pitch at robot startup
      // into initialPitch and use that as a reference for the
      // remainder of the autonomous or teleoperated session.

    double delta =  gyro.getPitch() - initialPitch;

    return delta;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IMU_Connected", gyro.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", gyro.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
    SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());
    /*
    SmartDashboard.putNumber("IMU_X", gyro.getDisplacementX());
    SmartDashboard.putNumber("IMU_Y", gyro.getDisplacementY());
    SmartDashboard.putNumber("IMU_Z", gyro.getDisplacementZ());
    SmartDashboard.putNumber("IMU_VX", gyro.getVelocityX());
    SmartDashboard.putNumber("IMU_VY", gyro.getVelocityY());
    SmartDashboard.putNumber("IMU_VZ", gyro.getVelocityZ());
    */
    
    // According to the documentation for the NavX gyro it takes some
    // time to perform a self calibration. The robot runs this
    // periodic() function 50 times / second and so we put the
    // settings for initialPitch here and check to see that the
    // NavX is finished calibrating before asking for the current
    // pitch and storing it.

    if(! gyro.isCalibrating() && ! isPitchSet) {
      initialPitch = gyro.getPitch();
      isPitchSet = true;
      SmartDashboard.putNumber("IMU_Zero_Pitch", initialPitch);
      this.reset();
      gyro.resetDisplacement();
    }
  }

  public void reset() {
    gyro.zeroYaw();
  }
  public void resetDisplacement() {
    gyro.resetDisplacement();
  }
}
