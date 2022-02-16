// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraVisionSubsystem;
import org.photonvision.PhotonUtils;

public class CameraVisionCommand extends CommandBase {
  /** Creates a new CameraVisionCommand. */
  private final CameraVisionSubsystem m_cameraSubsystem;
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(32.5);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(6.0);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.0);
  final double GOAL_RANGE_METERS = Units.feetToMeters(0.5);

  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;

  PIDController forwardController;
  PIDController turnController;

  double forwardSpeed;
  double rotationSpeed;

  public CameraVisionCommand(CameraVisionSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cameraSubsystem = subsystem;
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    forwardSpeed = 0.01;
    rotationSpeed = 0.01;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_cameraSubsystem.hasTargets() ) {
      var results = m_cameraSubsystem.camera.getLatestResult();

        double range =
        PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(results.getBestTarget().getPitch()));
        forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

        rotationSpeed = -turnController.calculate(results.getBestTarget().getYaw(), 0);
    } else {
      forwardSpeed = 0.02;
      rotationSpeed = 0.02;
    }
    SmartDashboard.putNumber("Fwd speed", forwardSpeed);
    SmartDashboard.putNumber("Rot speed", rotationSpeed);
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
