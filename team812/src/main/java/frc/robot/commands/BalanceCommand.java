// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PidConstants;

public class BalanceCommand extends CommandBase {
  /** Creates a new BalanceCommand. */
  private final DriveTrain m_subsystem;
  private final GyroSubsystem m_gyro;

  public BalanceCommand(final DriveTrain subsystem, final GyroSubsystem gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double delta = m_gyro.getPitch();
    double ratio = delta < 0.0 ? PidConstants.kProportionalBalanceBackward : PidConstants.kPorportionalBalanceForward;
    // TODO: add I and D constants
    double balanceSpeed = MathUtil.clamp(delta * ratio, -0.25, 0.25);
    if (Math.abs(delta) < 0.1) balanceSpeed = 0.0;
    SmartDashboard.putNumber("bal-speed", balanceSpeed);
    SmartDashboard.putNumber("bal-delta", delta);
    SmartDashboard.putNumber("bal-proportion", ratio);
    m_subsystem.arcadeDrive(-balanceSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
