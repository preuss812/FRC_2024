// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.EncoderSubsystem;
import frc.robot.subsystems.BrakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PidConstants;

public class BalanceCommandDebugEZ2 extends CommandBase {
  /** Creates a new BalanceCommand. */
  private final DriveTrain m_subsystem;
  private final GyroSubsystem m_gyro;
  private final EncoderSubsystem m_encoder;
  private final BrakeSubsystem m_brake;      // Maybe this should be outside in a sequential command.
  private  double m_driveSpeed;        // Percent of drive (-1..1)
  private  double m_targetDistance;    // Distance to travel in inches. Expecting somthing around 80..82 inches based on field and robot dimensions.
  private  double m_distance;          // Travel since command initialized.
  private  double m_encoderReferenceLeft;
  private  double m_encoderReferenceRight;
  //private static double m_lastAdjustment;
  private  double m_targetAngle;
  //private final double kAdjust = 0.1;
  //private final double kVelocityCorrection = 0.1;

  
  public BalanceCommandDebugEZ2(final DriveTrain subsystem, final GyroSubsystem gyro, final EncoderSubsystem encoder, final BrakeSubsystem brake, final double distance, final double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_gyro = gyro;
    m_encoder = encoder;
    m_brake = brake;
    m_driveSpeed = distance >= 0 ? speed : -speed;
    m_targetDistance = distance;

    addRequirements(m_subsystem,m_gyro,m_encoder,m_brake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_encoderReferenceLeft  = m_encoder.getLeftNumberDist();
    m_encoderReferenceRight = m_encoder.getRightNumberDist();
    m_targetAngle = m_gyro.getAngle();
    m_distance = 0.0; // We havent moved yet.
    m_brake.unBrake();                 // make sure the brakes are not on.

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double currentPitch = m_gyro.getPitch();
    //double ratio = currentPitch < 0.0 ? PidConstants.kProportionalBalanceBackward : PidConstants.kPorportionalBalanceForward;
    double encoderLeft       = m_encoder.getLeftNumberDist();
    double encoderRight      = m_encoder.getRightNumberDist();
    //double encoderLeftSpeed  = m_encoder.getLeftNumberRate();
    //double encoderRightSpeed = m_encoder.getRightNumberRate();
    double deltaAngle        = m_targetAngle - m_gyro.getAngle();
    double turningValue      = MathUtil.clamp(deltaAngle * PidConstants.kProportionalDriveStraight, -1.0, 1.0);
    //double speedAdjustment   = 0.0;
    m_distance = ((encoderLeft+encoderRight) - (m_encoderReferenceLeft + m_encoderReferenceRight))/2.0;  // Averaging left + right
    
    SmartDashboard.putNumber("bal-turn", turningValue);
    SmartDashboard.putNumber("bal-dist", m_distance);

    //balanceSpeed = speed;
    m_subsystem.arcadeDrive(-m_driveSpeed, -turningValue);
    //m_lastPitch = currentPitch;
    //m_lastAdjustment = speedAdjustment; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.arcadeDrive(0,0); // stop the drive
    m_brake.brake();                             // apply the brakes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return true if we travelled the requested distance
    SmartDashboard.putNumber("bal-goal", m_targetDistance);
    return ((m_targetDistance >= 0 && m_distance >= m_targetDistance) || (m_targetDistance < 0 && m_distance <= m_targetDistance));
  }
}
