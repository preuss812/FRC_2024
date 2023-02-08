// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Optional;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.CameraConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  private PhotonCamera cam = new PhotonCamera(CameraConstants.kCamName);
  //private final AnalogGyro m_gyro = new AnalogGyro(0);
  private Optional<EstimatedRobotPose> prevEstimatedRobotPose;
  private PhotonPoseEstimator photonPoseEstimator;
  public PhotonVisionSubsystem() {
    //AprilTagFieldLayout.loadFromResource("/Users/daveharry/FRC/FRC_2023/davesOffice.json");
    try {
      // AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      //Forward Camera
      Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      // Construct PhotonPoseEstimator
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, cam, robotToCam);
    }
    catch(Exception e) {
      // Need to add code here to report that we could not load the field layout - dph - 2023-02-05
    }
    Pose2d initialPoseMeters = new Pose2d();
    //DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(Constants.k_DriveKinematics, m_gyro.getRotation2d(),0,0,initialPoseMeters );
 }
 public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  Optional<EstimatedRobotPose> newPose = photonPoseEstimator.update();
  return newPose;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //prevEstimatedRobotPose = getEstimatedGlobalPose(prevEstimatedRobotPose.);
    try {
      prevEstimatedRobotPose = getEstimatedGlobalPose(new Pose2d());

      if (prevEstimatedRobotPose == null) {
        System.out.println("PhotonVisionSubsystem::execute: no targets found");
      }
      else
      {
        Pose3d currentPose = prevEstimatedRobotPose.get().estimatedPose;
        if (currentPose == null) {
          System.out.println("PhotonVisionSubsystem::execute: had a pose but lost it");
        }
        else {
          SmartDashboard.putNumber("X", currentPose.getX());
          SmartDashboard.putNumber("Y", currentPose.getY());
          SmartDashboard.putNumber("Z", currentPose.getZ());
        }
      
      }
    }
   catch(Exception e) {
     //System.out.println("PhotonVisionSubsystem::execute: Caught exception, no targets found");
    }
     
    /*
     public void update(double leftDist, double rightDist) {
      m_poseEstimator.update(m_gyro.getRotation2d(), leftDist, rightDist);

      var res = cam.getLatestResult();
      if (res.hasTargets()) {
          var imageCaptureTime = res.getTimestampSeconds();
          var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
          var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
          m_poseEstimator.addVisionMeasurement(
                  camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
      }
   }
   */

  }

}
