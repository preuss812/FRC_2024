/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PCMConstants;
import frc.robot.Constants.CANConstants;

public class RampSubsystem extends SubsystemBase {
  /**
   * Creates a new RampSubsystem.
   */
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
    CANConstants.kPCM,
    PCMConstants.kLiftPistons[0],
    PCMConstants.kLiftPistons[1]
  );

  public RampSubsystem() {
    // define initial position to be taken when this subsystem is initialized
    down();
  }

  public void up() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void down() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
