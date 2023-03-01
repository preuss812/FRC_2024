// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  private final ArmRotationSubsystem m_armSubsystem;
  private final double m_position;
  private double setPoint;
  private boolean end_game;

  public ArmCommand(ArmRotationSubsystem subsystem, double position) {
    m_armSubsystem = subsystem;
    m_position = position;
    System.out.println("ArmCommand class setPoint is " + m_position);
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("armcmd", "started");

    // end_game = frc.robot.RobotContainer.m_ElevatorSubsystem.is_endgame();
    end_game = true; // There is no end game in 2023. this enables full range of motion.
    System.out.println("ArmCommand Initialize end_game is " + end_game);
    if (end_game) {
      setPoint = m_position;
      System.out.println("ArmCommand endgame is true, setPoint is " + setPoint);
    } else {
      setPoint = Math.min(m_position, ArmConstants.kArmEndGamePosition);
      System.out.println("ArmCommand endgame is false, setPoint is " + setPoint);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPosition(setPoint);   // TODO: Does this need to be here? - dph 2023-03-01
  }

  public boolean onTarget() {
    double error = m_armSubsystem.getPosition() - setPoint;
    SmartDashboard.putNumber("armcmderr", error);

    // add more threshold as the arm goes up
    // we think increasing acceptable error is better than adding a PID.I term
    // after 1000 ticks it will increase slightly as a multiplier of kArmThreshold
    double extraThreshold = Math.max(m_armSubsystem.getPosition() * 0.001 - 1, 1);

    if (Math.abs(error) < ArmConstants.kArmThreshold * extraThreshold) {
      return true;
    } else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("armcmd", "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget();
  }
}
