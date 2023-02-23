// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.commands.ArmExtensionCommand;
import frc.robot.commands.ArmCommand;

public class ArmRepositionCommand extends SequentialCommandGroup {
  public static ArmRotationSubsystem m_armRotationSubsystem;
  public static ArmExtensionSubsystem m_armExtensionSubsystem;
    /** Creates a new ArmRepositionCommand. */
  public ArmRepositionCommand(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem,double rotation, double extension) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armRotationSubsystem = armRotationSubsystem;
    m_armExtensionSubsystem = armExtensionSubsystem;
    SmartDashboard.putNumber("ArmReposition: Rotation",rotation);
    SmartDashboard.putNumber("ArmReposition: getPosition", m_armRotationSubsystem.getPosition("ABC"));
    if (rotation > m_armRotationSubsystem.getPosition("A")) {
      addCommands( new ArmCommand(m_armRotationSubsystem, rotation).withTimeout(5), new ArmExtensionCommand(m_armExtensionSubsystem, extension).withTimeout(5));
      SmartDashboard.putString("ArmRepositionBranch", "A going up");
    } else {
      addCommands( new ArmExtensionCommand(m_armExtensionSubsystem, extension));
      addCommands( new ArmCommand(m_armRotationSubsystem, rotation));
      SmartDashboard.putString("ArmRepositionBranch", "B going down");

    }
  }

  /*
// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  */
  
}
