/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PCMConstants;
import frc.robot.Constants.GripperConstants;

/**
 * Add your docs here.
 */
public class GripperSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private String gripperState = GripperConstants.kUnknown;

  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
      CANConstants.kPCM,
      PneumaticsModuleType.CTREPCM,
      PCMConstants.kGripper[0],
      PCMConstants.kGripper[1]);

  public GripperSubsystem() {
    // This is where you might set an initial state of open or closed
  }

  public void openGrip() {
    double pressure = frc.robot.RobotContainer.m_Compressor.get_pressure();
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    if (pressure > PCMConstants.kMinPresssure) {
      gripperState = GripperConstants.kOpen;
    } else {
      gripperState = GripperConstants.kUnknown;
    }
    SmartDashboard.putString("Gripper", gripperState);
  }

  public void closeGrip() {
    double pressure = frc.robot.RobotContainer.m_Compressor.get_pressure();
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    if (pressure > PCMConstants.kMinPresssure) {
      gripperState = GripperConstants.kClosed;
    } else {
      gripperState = GripperConstants.kUnknown;
    }
    SmartDashboard.putString("Gripper", gripperState);
  }

  public boolean isOpen() {
    return (gripperState == GripperConstants.kOpen);
  }

  public boolean isClosed() {
    return (gripperState == GripperConstants.kClosed);
  }
  /*
   * @Override
   * public void initDefaultCommand() {
   * // Set the default command for a subsystem here.
   * // setDefaultCommand(new MySpecialCommand());
   * }
   */

}
