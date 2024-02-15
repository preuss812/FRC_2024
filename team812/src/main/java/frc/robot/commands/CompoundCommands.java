// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;

public class CompoundCommands {
  
  public static SequentialCommandGroup ScoreNoteInAmp(ArmRotationSubsystem armRotationSubsystem, ShooterSubsystem shooterSubsystem) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmUP")),
      new ArmRotationCommand(armRotationSubsystem, ArmConstants.kArmHiPosition).withTimeout(3.0),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Shoot")),
      new ShooterCommand(shooterSubsystem, 0.5).withTimeout(1.0),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmDown")),
      new ArmRotationCommand(armRotationSubsystem, ArmConstants.kArmHiPosition).withTimeout(3.0),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ScoreNoteDone"))
    );
  }

  // Return Pose for robot to be in scoring position for the current Alliance's AMP.
  public static GotoPoseCommand gotoAmp(DriveSubsystemSRX driveTrain, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        Pose2d ampPose;

        if (Utilities.isBlueAlliance()) {
            ampPose = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.BLUE_AMP.id());
        } else if (Utilities.isRedAlliance()) {
            ampPose = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.RED_AMP.id());
        }
        else {
            ampPose = poseEstimatorSubsystem.getCurrentPose(); // Hack:: if we dont know the alliance. Dont move. 
        }

    return new GotoPoseCommand(poseEstimatorSubsystem, driveTrain, ampPose.getX(), ampPose.getY(), ampPose.getRotation().getRadians());

  }

  // Return Pose for robot to be facing the source for the current alliance.
  public static GotoPoseCommand gotoSource(DriveSubsystemSRX driveTrain, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    Pose2d ampPose;

    if (Utilities.isBlueAlliance()) {
        ampPose = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.BLUE_RIGHT_SOURCE.id());
    } else if (Utilities.isRedAlliance()) {
        ampPose = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.RED_LEFT_SOURCE.id());
    }
    else {
        ampPose = poseEstimatorSubsystem.getCurrentPose(); // Hack:: if we dont know the alliance. Dont move. 
    }

    return new GotoPoseCommand(poseEstimatorSubsystem, driveTrain, ampPose.getX(), ampPose.getY(), ampPose.getRotation().getRadians());
  }

}
