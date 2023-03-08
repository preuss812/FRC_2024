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
  private double setPoint;  // TODO get rid of setPoint and use m_armExtensionSubsystem.getTargetPosition or getPositionError
  private int m_counter;

  public ArmExtensionCommand(ArmExtensionSubsystem subsystem, double position) {
    m_armExtensionSubsystem = subsystem;
    m_position = position;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_counter = 0;
    SmartDashboard.putString("armExtensionCmd", "started");

    setPoint = m_position;
    m_armExtensionSubsystem.setPosition(setPoint);
    // System.out.println("ArmExtension, setPoint is " + setPoint);
    // System.out.println("ArmExtensionCommand class setPoint is " + m_position);
    SmartDashboard.putNumber("ArmExtension: Goal position",m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //  m_armExtensionSubsystem.setPosition(setPoint); // TODO Verify this is not needed
   SmartDashboard.putNumber("armExtendExCalled", ++m_counter);
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
    //return false;
    return onTarget();
  }
}
