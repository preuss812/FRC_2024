// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CameraVisionSubsystem;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.subsystems.DriveTrain;

public class CameraVisionPoseCommand extends CommandBase {
  /** Creates a new CameraVisionCommand. */
  private final CameraVisionSubsystem m_cameraSubsystem;
  private final PhotonPoseEstimator m_photonPoseEstimator;
  private final DriveTrain m_drivetrainSubsystem;

  private EstimatedRobotPose ref_pose;

  public CameraVisionPoseCommand(CameraVisionSubsystem subsystem, DriveTrain drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    AprilTagFieldLayout atfl = null;
    final AprilTag tag01 =
      new AprilTag(
              0,
              new Pose3d(new Pose2d(0.0, FieldConstants.width / 2.0, Rotation2d.fromDegrees(0.0))));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag01);

    atfl = new AprilTagFieldLayout(
      atList,
      FieldConstants.length,
      FieldConstants.width
    );

    m_cameraSubsystem = subsystem;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_cameraSubsystem.camera, VisionConstants.robotToCam);
    addRequirements(subsystem, drivetrainSubsystem);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d estimatedPose) {
    m_photonPoseEstimator.setReferencePose(estimatedPose);
    return m_photonPoseEstimator.update();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose3d initial_ref_pose = new Pose3d();
    ref_pose = getEstimatedGlobalPose(initial_ref_pose).get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> estrp = getEstimatedGlobalPose(ref_pose.estimatedPose);
    if (estrp.isEmpty()) {
      return;
    }
    Pose3d estimated_pose = estrp.get().estimatedPose;
    ref_pose = estrp.get();
    SmartDashboard.putNumber("X", estimated_pose.getX());
    SmartDashboard.putNumber("Y", estimated_pose.getY());
    SmartDashboard.putNumber("Z", estimated_pose.getZ());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}