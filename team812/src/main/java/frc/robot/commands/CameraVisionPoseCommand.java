// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    // Use addRequirements() here to declare subsystem dependencies.
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

    AprilTagFieldLayout atfl = layout;
    m_fieldLayout = layout;

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




    // Update pose estimator with the best visible target
    var target = m_cameraSubsystem.getBestTarget();

    if (target != null) {

      var fiducialId = target.getFiducialId();
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      Optional<Pose3d> tagPose = m_fieldLayout == null ? Optional.empty() : m_fieldLayout.getTagPose(fiducialId);
      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        // var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        var resultTimestamp = System.currentTimeMillis() / 1000;
        m_photonPoseEstimator.addVisionMeasurement(camPose.toPose2d(), resultTimestamp);
    }


    // Update pose estimator with drivetrain sensors
    // m_photonPoseEstimator.update(
    //   drivetrainSubsystem.getGyroscopeRotation(),
    //   drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(m_photonPoseEstimator.update());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
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






