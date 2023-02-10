// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;

public class TurnRightBB2 extends CommandBase {
  /** Creates a new TrunRightBB2. */
  private final DriveTrain driveTrain;

  public TurnRightBB2 (final DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      driveTrain = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, 0.0, 1.0);

    SmartDashboard.putNumber ("TRBBRotation", rotation);

          driveTrain.preussDrive(0.0, rotation) ;
    
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
