// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.Constants.ArmExtensionConstants;

public class ArmExtensionCommand extends CommandBase {
  /** Creates a new ArmExtensionCommand. */
  private final ArmExtensionSubsystem m_armExtensionSubsystem;
  private final double m_position;
  private double setPoint;
  private boolean end_game;

  public ArmExtensionCommand(ArmExtensionSubsystem subsystem, double position) {
    m_armExtensionSubsystem = subsystem;
    m_position = position;
    System.out.println("ArmExtensionCommand class setPoint is " + m_position);
    SmartDashboard.putNumber("ArmExtension: Goal position",position);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("armExtensionCmd", "started");

    // end_game = frc.robot.RobotContainer.m_ElevatorSubsystem.is_endgame();
    end_game = true; // There is no end game in 2023. this enables full range of motion.
    System.out.println("ArmExtensionCommand Initialize end_game is " + end_game);
      setPoint = m_position;
      // System.out.println("ArmExtension, setPoint is " + setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armExtensionSubsystem.setPosition(setPoint);
  }

  public boolean onTarget() {
    double error = m_armExtensionSubsystem.getPosition() - setPoint;
    SmartDashboard.putNumber("armExtensionCmderr", error);
    if (Math.abs(error) < ArmExtensionConstants.kArmExtensionThreshold) {
      return true;
    } else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("armExtensionCmd", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget();
  }
}
