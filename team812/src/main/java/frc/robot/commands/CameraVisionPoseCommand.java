// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/apriltagExample/src/main/java/frc/robot/Drivetrain.java

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private AprilTagFieldLayout m_fieldLayout;
  private final Field2d field2d = new Field2d();


  private EstimatedRobotPose ref_pose;

  public CameraVisionPoseCommand(CameraVisionSubsystem subsystem, DriveTrain drivetrainSubsystem) {
    // Load april tag field
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance == Alliance.Blue ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }

    m_fieldLayout = layout;
    SmartDashboard.putData("Field", field2d);


    m_cameraSubsystem = subsystem;
    m_drivetrainSubsystem = drivetrainSubsystem;
    // set pose estimator and PoseStrategy.CLOSEST_TO_REFERENCE_POSE
    m_photonPoseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_cameraSubsystem.camera, VisionConstants.robotToCam);
    addRequirements(subsystem, drivetrainSubsystem);
    
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d estimatedPose) {
    m_photonPoseEstimator.setReferencePose(estimatedPose);
    return m_photonPoseEstimator.update();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Pose3d initial_ref_pose = new Pose3d();
    // ref_pose = getEstimatedGlobalPose(initial_ref_pose).get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // getEstimatedGlobalPose(m_photonPoseEstimator.getEstimatedPosition());
    try {
      Pose2d estimatedPose2d = (m_photonPoseEstimator.update().get().estimatedPose.toPose2d());
      SmartDashboard.putString("Position estimated", getFomattedPose(estimatedPose2d));
      if (m_photonPoseEstimator.update().isPresent()) {
        field2d.setRobotPose(estimatedPose2d);
      }
    }
    catch (Exception e) {
      // System.out.println("pose is null");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private String getFomattedPose(Pose2d pose) {
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation. Assumes a planar field and the robot is always firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return m_photonPoseEstimator.update();
  }
}

