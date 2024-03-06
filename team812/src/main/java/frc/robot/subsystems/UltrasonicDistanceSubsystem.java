// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UltrasonicDistanceSubsystem extends SubsystemBase {
  private final Ultrasonic ultrasonicSensor;

  /** Creates a new UltrasonicDistanceSubsystem. */
  public UltrasonicDistanceSubsystem(int pingChannel, int echoChannel) {
    this.ultrasonicSensor = new Ultrasonic(pingChannel, echoChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("US mm"+ultrasonicSensor.getEchoChannel(), ultrasonicSensor.getRangeMM());
  }
  /**
   * return the range to the nearest object to the sensor.
   * @return
   */
  public double getRange() {
    return ultrasonicSensor.getRangeMM()/1000.0;
  }
}
