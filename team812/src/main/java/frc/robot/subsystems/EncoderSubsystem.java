/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.EncoderConstants;
// Java Reference Document for the Encoder class
// https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Encoder.html

public class EncoderSubsystem extends SubsystemBase {

  private Encoder p_right_gearbox_encoder, p_left_gearbox_encoder;

  public EncoderSubsystem() {
    // This is the object constructor
    
    p_right_gearbox_encoder = new Encoder(EncoderConstants.kRightDriveEncoder[0],
                                          EncoderConstants.kRightDriveEncoder[1],
                        false,
                                          Encoder.EncodingType.k2X);

    p_left_gearbox_encoder = new Encoder(EncoderConstants.kLeftDriveEncoder[0],
                                         EncoderConstants.kLeftDriveEncoder[1],
                      false,
                                        Encoder.EncodingType.k2X);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Left", getLeftNumberDist());
    SmartDashboard.putNumber("Encoder Right", getRightNumberDist());
  }

  public void AllReset() {
    LeftReset();
    RightReset();
  }

  // Right Gearbox Encoder methods
  public void RightReset() {
    p_right_gearbox_encoder.reset();
  }

  public double getRightNumberRate() {
    return p_right_gearbox_encoder.getRate();
  }

  public double getRightNumberDist() {
    return p_right_gearbox_encoder.getDistance();
  }

  public void setRightDistancePerPulse(double factor) {
    p_right_gearbox_encoder.setDistancePerPulse(factor);
  }
  
  
  // Left Gearbox Encoder methods
  public void LeftReset() {
    p_left_gearbox_encoder.reset();
  }

  public double getLeftNumberRate() {
    return p_left_gearbox_encoder.getRate();
  }

  public double getLeftNumberDist() {
    return p_left_gearbox_encoder.getDistance();
  }

  public void setLeftDistancePerPulse(double factor) {
    p_left_gearbox_encoder.setDistancePerPulse(factor);
  }

}
