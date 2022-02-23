// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraVisionSubsystem;
import org.photonvision.PhotonUtils;
import frc.robot.subsystems.DriveTrain;

public class CameraVisionCommand extends CommandBase {
  /** Creates a new CameraVisionCommand. */
  private final CameraVisionSubsystem m_cameraSubsystem;
  private final DriveTrain m_drivetrainSubsystem;

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(32.5);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(6.0);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.0);
  final double GOAL_RANGE_METERS = Units.feetToMeters(0.5);

  final double LINEAR_P = 0.0;
  final double LINEAR_D = 0.0;
  final double ANGULAR_P = 0.8;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0;

  PIDController forwardController;
  PIDController turnController;

  double forwardSpeed;
  double rotationSpeed;

  public CameraVisionCommand(CameraVisionSubsystem subsystem, DriveTrain drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cameraSubsystem = subsystem;
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(subsystem, drivetrainSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    turnController.setTolerance(4.0); // did not work, dont understand yet
    forwardSpeed = 0.01;
    rotationSpeed = 0.01;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error;
    if (m_cameraSubsystem.hasTargets() ) {
      var results = m_cameraSubsystem.camera.getLatestResult();

        double range =
        PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(results.getBestTarget().getPitch()));
        forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
        forwardSpeed= 0.0;
        rotationSpeed = MathUtil.clamp(-turnController.calculate(results.getBestTarget().getYaw(), 0),-0.50,0.50);
        error = turnController.getPositionError();
        SmartDashboard.putNumber("Turn error", error);
    } else {
      forwardSpeed = 0.00;
      rotationSpeed = 0.00;
    }
    SmartDashboard.putNumber("Fwd speed", forwardSpeed);
    SmartDashboard.putNumber("Rot speed", rotationSpeed);
   m_drivetrainSubsystem.drive(forwardSpeed, rotationSpeed); 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
