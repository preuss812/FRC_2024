/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

//import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PCMConstants;
import frc.robot.Constants.BrakeConstants;

/**
 * Add your docs here.
 */
public class BrakeSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private String brakeState = BrakeConstants.kUnknown;

  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
      CANConstants.kPCM,
      PneumaticsModuleType.CTREPCM,
      PCMConstants.kBrake[0],
      PCMConstants.kBrake[1]);
  private final BrakeLight m_brakeLight = new Relay(Constants.kBrakeLightRelay, Relay.Direction.kForward);

  public BrakeSubsystem() {
      unBrake();
    // This is where you might set an initial state of open or closed
  }

  public void unBrake() {
    double pressure = frc.robot.RobotContainer.m_Compressor.get_pressure();
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    m_brakeLight.set(Value.kOff);
    if (pressure > PCMConstants.kMinPresssure) {
      brakeState = BrakeConstants.kNotBraking;
    } else {
      brakeState = BrakeConstants.kUnknown;
    }
    SmartDashboard.putString("Brake", brakeState);
  }

  public void brake() {
    double pressure = frc.robot.RobotContainer.m_Compressor.get_pressure();
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    m_brakeLight.set(Value.kOn);
    if (pressure > PCMConstants.kMinPresssure) {
      brakeState = BrakeConstants.kBraking;
    } else {
      brakeState = BrakeConstants.kUnknown;
    }
    SmartDashboard.putString("Brake", brakeState);
  }

  public boolean isNotBraking() {
    return (brakeState == BrakeConstants.kNotBraking);
  }

  public boolean isBraking() {
    return (brakeState == BrakeConstants.kBraking);
  }
  /*
   * @Override
   * public void initDefaultCommand() {
   * // Set the default command for a subsystem here.
   * // setDefaultCommand(new MySpecialCommand());
   * }
   */

}
