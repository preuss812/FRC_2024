// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

public class CameraLightSubsystem extends SubsystemBase {
  /** Creates a new CameraLightSubsystem. */
  private final Relay m_lightring = new Relay(Constants.kLightRelay, Relay.Direction.kForward);

  public CameraLightSubsystem() {}

  public void on() {
    m_lightring.set(Value.kOn);
  }

  public void off(){
    m_lightring.set(Value.kOff);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
