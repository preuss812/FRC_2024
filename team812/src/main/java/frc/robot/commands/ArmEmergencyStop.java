// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

// This command simply cancels any active arm comand and tells the arm rotation and extension to stay where they currently are.
public class ArmEmergencyStop extends CommandBase {
  /** Creates a new ArmCommand. */
  
  private final ArmRotationSubsystem m_armRotationSubsystem;
  private final ArmExtensionSubsystem m_armExtensionSubsystem;

  public ArmEmergencyStop(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
    m_armRotationSubsystem = armRotationSubsystem;
    m_armExtensionSubsystem = armExtensionSubsystem;
    addRequirements(armRotationSubsystem);
    addRequirements(armExtensionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("*** Arm Emergency STOP");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armRotationSubsystem.setPosition(m_armRotationSubsystem.getPosition()); // TODO Shouldnt this be in initialize()? - dph 2023-03-01
    m_armExtensionSubsystem.setPosition(m_armExtensionSubsystem.getPosition()); // TODO Shouldnt this be in initialize()? - dph 2023-03-01
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
