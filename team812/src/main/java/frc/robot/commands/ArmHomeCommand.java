// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
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
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (l_pressure >= PCMConstants.kMinPresssure) {
      m_armExtensionSubsystem.setSensorPosition(ArmExtensionConstants.kArmExtensionReferencePosition);  // Tell the arm extension subsystem it is currently at the reference/home Position.class
      m_armExtensionSubsystem.setHomePosition(ArmExtensionConstants.kArmExtensionReferencePosition);    // Tell the arm extention subsystem to stay where it is/
      m_armExtensionSubsystem.setHome();                                                      
      m_armSubsystem.setSensorPosition(ArmConstants.kArmAutonomousReferencePosition);
      m_armSubsystem.setHomePosition(ArmConstants.kArmAutonomousReferencePosition); // Cant really do this safely until the arms are retracted. - dph
      m_armSubsystem.setHome();
    }
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
    if (l_pressure >= PCMConstants.kMinPresssure &&
        m_armExtensionSubsystem.isHome() &&
        m_armSubsystem.isHome()) {
      return true; // Enthe command because we cannot operation without pressure to close the
    }
    return false;
  }
}
