/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.AnalogIOConstants;

public class CompressorSubsystem extends SubsystemBase {
  /**
   * Creates a new CompressorSubsystem.
   */
  private final Compressor m_compressor = new Compressor(CANConstants.kPCM);
  private final AnalogPotentiometer m_pressure = new AnalogPotentiometer(
    AnalogIOConstants.kPressureTransducer,
    AnalogIOConstants.kPressureRange,
    AnalogIOConstants.kPressureOffset
  );


  public CompressorSubsystem() {
    m_compressor.start();

  }

  public double get_pressure() {
    final double pressure = m_pressure.get();
    SmartDashboard.putNumber("Pressure is ", pressure);
    return pressure;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
