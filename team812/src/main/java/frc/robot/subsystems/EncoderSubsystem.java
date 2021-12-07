/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

public class EncoderSubysystem extends SubsystemBase {

  private final Encoder p_encoder = new Encoder(8,9,false,Encoder.EncodingType.k2X);

  public EncoderSubysystem() {}

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
}
