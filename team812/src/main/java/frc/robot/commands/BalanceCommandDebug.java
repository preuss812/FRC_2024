// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.EncoderSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

public class BalanceCommandDebug extends CommandBase {
  /** Creates a new BalanceCommand. */
  private final DriveTrain m_subsystem;
  private final GyroSubsystem m_gyro;
  private final EncoderSubsystem m_encoder;
  private static double m_lastPitch = 0.0;

  public BalanceCommandDebug(final DriveTrain subsystem, final GyroSubsystem gyro, final EncoderSubsystem encoder) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_gyro = gyro;
    m_encoder = encoder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = m_gyro.getPitch();
    double ratio = currentPitch < 0.0 ? PidConstants.kProportionalBalanceBackward : PidConstants.kPorportionalBalanceForward;
    double extraPower = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, -0.1, 0.1);
    extraPower = 0.026; // this was too much
    extraPower = 0.018; // better but still too much;
    extraPower = 0.014; // A little too lo but just barely failed
    extraPower = 0.016; // This was perfect but the next 3 games it was too little.
    extraPower = 0.020; // a guess 

    double balanceSpeed = MathUtil.clamp(currentPitch * ratio, -0.25-extraPower, 0.25+extraPower);
    double deltaPitch = currentPitch - m_lastPitch;
    Integer balancePath = 0;
    SmartDashboard.putNumber("dither",extraPower);
    
    // TODO: add I and D constants
    if (currentPitch < -2.0) {
        if (deltaPitch < -0.01) {
            // Use default balanceSpeed, we are backing up toward the center.
            balancePath = 1;
        } else if (deltaPitch > 0.1) {
            // We are reducing pitch so we are approaching or possibly past the balance point
            // Reverse a little.
            balanceSpeed = 0.05;
            balancePath = 2;
        } else {
            balancePath = 3;
        }
    } else if (currentPitch > 2.0) {
        if (deltaPitch > 0.01) {
            // Use default balanceSpeed, we are moving forward up toward the center.
            balancePath = 4;
        } else if (deltaPitch > 0.1) {
            // We are reducing pitch so we are approaching or possibly past the balance point
            // Reverse a little.
            balancePath = 5;
            balanceSpeed = -0.05;
        } else {
            balancePath=6;
        }
    } else {
        // Current Pitch is between -2 and +2 degrees, that's basically balanced.
        // The precalculated balance speed will be 0.1 or less.
        // Just use that value.
        balancePath = 6;
    }
    if (Math.abs(currentPitch) < 0.1) balanceSpeed = 0.0;
    SmartDashboard.putNumber("bal-speed", balanceSpeed);
    SmartDashboard.putNumber("bal-pitch", currentPitch);
    SmartDashboard.putNumber("bal-delta", deltaPitch);
    SmartDashboard.putNumber("bal-proportion", ratio);
    SmartDashboard.putNumber("bal-path", balancePath);
    m_subsystem.arcadeDrive(-balanceSpeed, 0);
    m_lastPitch = currentPitch;
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
