// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This command drives the robot to the AMP for the robot's alliance.
 * It will eventually follow a pre-planned trajectory from anywhere on the
 * field to the AMP.  It should probably lower the arm and quiese the intake
 * shooter motors as well.
 */
public class GotoAmpCommand extends SequentialCommandGroup {
  /** Creates a new GotoAmpCommand. */
  public GotoAmpCommand(DriveSubsystemSRX driveTrain, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    Pose2d ampPose;

        if (Utilities.isBlueAlliance()) {
            ampPose = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.BLUE_AMP.id());
        } else if (Utilities.isRedAlliance()) {
            ampPose = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.RED_AMP.id());
        }
        else {
            ampPose = poseEstimatorSubsystem.getCurrentPose(); // Hack:: if we dont know the alliance. Dont move. 
        }

    addCommands(new GotoPoseCommand(poseEstimatorSubsystem, driveTrain, ampPose));
  }
}
