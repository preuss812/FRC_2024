// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

// FOR DEBUG:
// This class causes the robot to turn based on the Black Box X Pot setting.
// Intended to allow Characterization of output values vs rotation speeds of the robot.
// The pot is read scaled from -1 to 1 where negative readings result in left turns and positive readings result in right turns.
public class TurnRightBB extends CommandBase {
  /** Creates a new TurnRightBB. */
  private final DriveTrain driveTrain;

  public TurnRightBB (final DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      driveTrain = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, -1.0, 1.0);
    SmartDashboard.putNumber("TRBB rotation", rotation);
    driveTrain.preussDrive(0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.preussDrive(0.0, 0.0) ;  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}