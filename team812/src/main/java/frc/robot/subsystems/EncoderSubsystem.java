/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

// Java Reference Document for the Encoder class
// https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Encoder.html

public class EncoderSubsystem extends SubsystemBase {

  private Encoder p_encoder;

  public EncoderSubsystem() {
    // This is the object constructor
    p_encoder = new Encoder(8,9,false,Encoder.EncodingType.k2X);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getNumberRate() {
    return p_encoder.getRate();
  }

  public double getNumberDist() {
    return p_encoder.getDistance();
  }

  // Would recommend creating methods for the following
  // getStopped
  // setDistancePerPulse
  // getDistancePerPulse
  // reset
}
