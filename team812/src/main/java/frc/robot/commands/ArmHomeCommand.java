// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.Constants.PCMConstants;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ArmHomeCommand extends CommandBase {
  /** Creates a new ArmHomeCommand. */
  private final ArmRotationSubsystem m_armSubsystem;
  private final ArmExtensionSubsystem m_armExtensionSubsystem;
  double l_pressure;

  public ArmHomeCommand(ArmRotationSubsystem subsystem, ArmExtensionSubsystem armExtensionSubsystem) {
    m_armSubsystem = subsystem;
    m_armExtensionSubsystem = armExtensionSubsystem;
    addRequirements(subsystem);
    addRequirements(armExtensionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    l_pressure = frc.robot.RobotContainer.m_Compressor.get_pressure();
    System.out.println("Home command STARTED");
    System.out.println("Home pressure: " + l_pressure);

    SmartDashboard.putString("homearm", "starting");
    if (l_pressure >= PCMConstants.kMinPresssure) {
      m_armExtensionSubsystem.setSensorPosition(ArmExtensionConstants.kArmExtensionFullyExtendedPosition);
      m_armExtensionSubsystem.setHomePosition(0);
      m_armSubsystem.setSensorPosition(4000.0);
      m_armSubsystem.setHomePosition(0.0); // Cant really do this safely until the arms are retracted. - dph
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("homearm", "end");
    System.out.println("Home command ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (l_pressure < PCMConstants.kMinPresssure) {
      return true; // End the command because we cannot operation without pressure to close the
                   // hands - dph
    } else {
      if (!m_armExtensionSubsystem.isHome() && m_armExtensionSubsystem.isOutLimitSwitchClosed()) { // Should isHome
                                                                                                      // be checked? -
                                                                                                      // dph
        m_armExtensionSubsystem.setSensorPosition(0.0);
        m_armExtensionSubsystem.setHomePosition(0.0); // Tell PID to keep arm at 0.
        m_armExtensionSubsystem.setHome();
      }
      if (!m_armSubsystem.isHome() && m_armSubsystem.isBottomLimitSwitchClosed()) {
        m_armSubsystem.setSensorPosition(0.0);
        m_armSubsystem.setHomePosition(0.0); // Tell PID to keep arm at 0.
        m_armSubsystem.setHome();
      } else {
        return false; // Arm extension is homed but the arm rotation is still homing.
      }
    }
    return m_armExtensionSubsystem.isHome() && m_armSubsystem.isHome();
  }
}
