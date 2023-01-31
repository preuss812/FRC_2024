// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnRight extends CommandBase {
  /** Creates a new TurnRight. */
  private final DriveTrain driveTrain;

  public TurnRight (final DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      driveTrain = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double jantLeft = 0.75; 
    double jantRight = -0.7;
    boolean doneTurning = false;
  
        driveTrain.tankDrive(jantLeft, jantRight) ;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0.0, 0.0) ;  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
