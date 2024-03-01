// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ForkJoinPool;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class SwerveToPoseCommand extends SequentialCommandGroup {
  
  /** Creates a new SwerveToPoseCommand. */
  public SwerveToPoseCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    String destination
  ) {
    Pose2d startingPose = poseEstimatorSubsystem.getCurrentPose();
    List<Translation2d> waypoints = new ArrayList<>();
    if (true) { //destination == "AMP" && Utilities.isBlueAlliance()) {
      Utilities.toSmartDashboard("SW Start",startingPose);
      waypoints = Utilities.planBlueAmpTrajectory(startingPose);
      SmartDashboard.putString("blueampplan", waypoints.toString());
      Pose2d aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_AMP.id());
      Pose2d nearTargetPose = Utilities.backToPose(aprilTagPose, 1.5);
      Pose2d targetPose = Utilities.backToPose(aprilTagPose, 0.3);
      Utilities.toSmartDashboard("SW target", nearTargetPose);
// Fast drive with loose X,Y,theta
      addCommands(new FollowTrajectoryCommand(robotDrive, poseEstimatorSubsystem, null, startingPose, waypoints, nearTargetPose));
      // Go to exact position.
      //addCommands(new GotoPoseCommand(poseEstimatorSubsystem, robotDrive, targetPose));
    } else if (destination == "AMP" && Utilities.isRedAlliance()) {
    } else if (destination == "SOURCE" && Utilities.isBlueAlliance()) {
    } else if (destination == "SOURCE" && Utilities.isRedAlliance()) {
    } else {
    }
  }
}
