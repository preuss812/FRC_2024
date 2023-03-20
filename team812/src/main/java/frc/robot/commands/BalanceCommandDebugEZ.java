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

public class BalanceCommandDebugEZ extends CommandBase {
  /** Creates a new BalanceCommand. */
  private final DriveTrain m_subsystem;
  private final GyroSubsystem m_gyro;
  private final EncoderSubsystem m_encoder;
  private final BrakeSubsystem m_brake; // Maybe this should be outside in a sequential command.
  private static double m_lastPitch = 0.0;
  private static double m_encoderReferenceLeft;
  private static double m_encoderReferenceRight;
  private static double m_lastAdjustment;
  private static double m_targetAngle;
  private final double kAdjust = 0.1;
  private final double kVelocityCorrection = 0.1;

  private double optimalVelocity(double incline, double distanceSoFar) {
    double nominalVelocity = 12.0; // inches per second.
    double result = 0.0;
    if (incline >= 2.0 ) {
        // We are pointed up on the ramp.
        if (distanceSoFar < 24.0) {
            result = nominalVelocity;
        } else if (distanceSoFar >= 24 && distanceSoFar < 30.0) {
            result = nominalVelocity * (30-distanceSoFar)/6.0; // Linear ramp down to 0.
        } else {
            result = 0.0;  // We think we are past the balance point.
        }
    } else if (incline < -2.0) {
        // We are pointed down on the ramp.
        if (distanceSoFar > 36) {
            result = -nominalVelocity;
        } else if (distanceSoFar >= 30.0 && distanceSoFar < 36.0) {
            result = nominalVelocity * (30-distanceSoFar)/6.0; // Linear ramp up to 0.
        } else {
            result = 0;
        }
    } else {
        // We are very near the balance point
        result = 0.0;
    }
    return result;
  }
  public BalanceCommandDebugEZ(final DriveTrain subsystem, final GyroSubsystem gyro, final EncoderSubsystem encoder, final BrakeSubsystem brake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_gyro = gyro;
    m_encoder = encoder;
    m_brake = brake;
    addRequirements(m_subsystem,m_gyro,m_encoder,m_brake);
    //SmartDashboard.putString("BALEZ Constructed", "TRUE");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //SmartDashboard.putString("BALEZ INIT", "TRUE");
    m_encoderReferenceLeft  = m_encoder.getLeftNumberDist();
    m_encoderReferenceRight = m_encoder.getRightNumberDist();
    m_targetAngle = m_gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = m_gyro.getPitch();
    double ratio = currentPitch < 0.0 ? PidConstants.kProportionalBalanceBackward : PidConstants.kPorportionalBalanceForward;
    double encoderLeft       = m_encoder.getLeftNumberDist();
    double encoderRight      = m_encoder.getRightNumberDist();
    double encoderLeftSpeed  = m_encoder.getLeftNumberRate();
    double encoderRightSpeed = m_encoder.getRightNumberRate();
    double deltaAngle        = m_targetAngle - m_gyro.getAngle();
    double turningValue      = MathUtil.clamp(deltaAngle * PidConstants.kProportionalDriveStraight, -1.0, 1.0);
    double speedAdjustment   = 0.0; 
    double distanceSoFar     = ((encoderLeft+encoderRight) - (m_encoderReferenceLeft + m_encoderReferenceRight))/2.0;  // Averaging left + right
    double deltaPitch        = currentPitch - m_lastPitch;
    double optimalVelocity   = optimalVelocity(currentPitch, distanceSoFar); // inches per second TODO Check units.
    double velocity          =  (encoderLeftSpeed+encoderRightSpeed)/2.0;
    Integer balancePath      = 0;
    double extraPower        = 0.0;
    final double maxPower    = 0.75;
    //SmartDashboard.putNumber("dither",extraPower);
    
    if (velocity > 0.0 && optimalVelocity > 0.0)
    {
        // using a so-called exponential adjustment
        // A(i+1) = A(i)*k + A(i-1)*(1-k) where k between 0 and 1.
        speedAdjustment = (velocity-optimalVelocity) * kVelocityCorrection;
        extraPower = (speedAdjustment)*kAdjust + m_lastAdjustment*(1.0-kAdjust); 
    }

    double balanceSpeed = MathUtil.clamp(currentPitch * ratio + extraPower, -maxPower, maxPower);

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
    SmartDashboard.putNumber("bal-turn", turningValue);
    SmartDashboard.putNumber("bal-V", velocity);
    SmartDashboard.putNumber("bal-optV", optimalVelocity);
    SmartDashboard.putNumber("bal-extra", extraPower);
    SmartDashboard.putNumber("bal-adj", m_lastAdjustment);
    SmartDashboard.putNumber("bal-dist", distanceSoFar);
     System.out.printf("BALEZ: %f %f %f %f %d %f %f %f %f %f %f\n"
        ,balanceSpeed
        ,currentPitch
        ,deltaPitch
        ,ratio
        ,balancePath
        ,turningValue
        ,velocity
        ,optimalVelocity
        ,extraPower
        ,m_lastAdjustment
        ,distanceSoFar
        );

    balanceSpeed = 0.55;
    m_subsystem.arcadeDrive(-balanceSpeed, -turningValue);
    m_lastPitch = currentPitch;
    m_lastAdjustment = speedAdjustment; 
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
    double encoderLeft       = m_encoder.getLeftNumberDist();
    double encoderRight      = m_encoder.getRightNumberDist();
    double encoderLeftSpeed  = m_encoder.getLeftNumberRate();
    double encoderRightSpeed = m_encoder.getRightNumberRate();
    double deltaAngle        = m_targetAngle - m_gyro.getAngle();
    double turningValue      = MathUtil.clamp(deltaAngle * PidConstants.kProportionalDriveStraight, -1.0, 1.0);
    double speedAdjustment   = 0.0; 
    double distanceSoFar     = ((encoderLeft+encoderRight) - (m_encoderReferenceLeft + m_encoderReferenceRight))/2.0;  // Averaging left + right
    //return false;
    return distanceSoFar >= 82; // 80 inches from start to center of gravity robot == center of platform.
  }
}
