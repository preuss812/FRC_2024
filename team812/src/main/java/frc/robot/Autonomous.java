/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.*;
import frc.robot.commands.ArmHomeCommand;
import frc.robot.commands.DriveRobotCommand;
import frc.robot.commands.FindAprilTagCommand;
import frc.robot.commands.ScoreNoteInAmp;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.RotateRobotCommand;
import frc.robot.commands.StopRobotMotion;
import frc.robot.commands.SwerveToPoseCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTag;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  private final RobotContainer m_robotContainer;
  private final DriveSubsystemSRX m_robotDrive;
  private final ArmRotationSubsystem m_ArmRotationSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
  // public GyroSubsystem m_gyro;

  // public Autonomous(DriveTrain subsystem, GyroSubsystem gyro) {
  public Autonomous(RobotContainer robotContainer) {
    //boolean balancing = true;
    //boolean readingBlackBoxSwitch = false;

    m_robotContainer = robotContainer;
    m_robotDrive = RobotContainer.m_robotDrive;
    m_ArmRotationSubsystem = RobotContainer.m_ArmRotationSubsystem;
    m_ShooterSubsystem = RobotContainer.m_ShooterSubsystem;
    m_PoseEstimatorSubsystem = RobotContainer.m_PoseEstimatorSubsystem;
    enum AutonomousStrategy {
      LABTEST,
      BLUEALLIANCE,
      REDALLIANCE,
      USEALLIANCE
    }
    // This is now unused mostly but left in in case we need it for testing.
    AutonomousStrategy autonomousStrategy = AutonomousStrategy.USEALLIANCE;

  
    if (autonomousStrategy == AutonomousStrategy.USEALLIANCE) {
      /**
       * We are starting with the robots back to the Alliance wall.
       * We hope we can be positioned anywhere along that wall.
       * The command thinks in terms of field coordinates with Blue X= 0 and Red at X=16-ish.
       * Units are in meters and radians.
       * The sequence of steps is:
       * o Set the gyro angle (0 for blue, 180 for red)
       * o Home the arm.
       * o Drive forward 1 meter (+1 meter for Blue, -1 meter for Red).
       * o Turn toward the back of the robot toward the AMP (-90 for Blue, +90 for Red)
       * o Find an april tag (rotate slowly until one is found).  Ideally, seen immediately.
       * o SwerveDrive toward the Amp (if we are close, this is a NO-OP).
       * o GotoPoseDrive to the Amp
       * o Score the Note.
       * o GotoPoseDrive out of the starting box toward field center.
       */
      final double distanceToAmp = 0.5; // If it matters, these are Blue Alliance values.
      final double firstMoveX = 1.0;
      final double firstMoveY = 0.0;
      final double finalMoveX = 2.0;
      final double finalMoveY = -1.0;
      VisionConstants.AprilTag ampAprilTag;
      Pose2d targetPose;
      Pose2d finalPose;
      Rotation2d finalRotation;
      Pose2d firstMove;
      double robotInitialOrientation;
      if (Utilities.isBlueAlliance()) {
        robotInitialOrientation = FieldConstants.robotInitialOrientation;
        ampAprilTag = AprilTag.BLUE_AMP;
        targetPose = Utilities.backToPose(m_PoseEstimatorSubsystem.getAprilTagPose(ampAprilTag.id()),distanceToAmp);
        finalRotation = targetPose.getRotation(); // This will put thte back of the robot towards the april tag.
        finalPose = new Pose2d(targetPose.getX()+finalMoveX, targetPose.getY() + finalMoveY, finalRotation); // Pose for robot to be at the april tag.
        firstMove = new Pose2d(firstMoveX, firstMoveY, new Rotation2d(-Math.PI/2));
      } else /* Assuming !blue == red */ {
        robotInitialOrientation = FieldConstants.robotInitialOrientation + Math.PI; // Rotated 180.
        ampAprilTag = AprilTag.RED_AMP;
        targetPose = Utilities.backToPose(m_PoseEstimatorSubsystem.getAprilTagPose(ampAprilTag.id()),distanceToAmp);
        finalRotation = targetPose.getRotation(); // This will put thte back of the robot towards the april tag.
        finalPose = new Pose2d(targetPose.getX()-finalMoveX, targetPose.getY() + finalMoveY, finalRotation); // Pose for robot to be at the april tag.
        firstMove = new Pose2d(-firstMoveX, firstMoveY, new Rotation2d(Math.PI/2));
      }
      Utilities.toSmartDashboard("AutoTarget", targetPose);
      Utilities.toSmartDashboard("AutoFinal", finalPose);

      SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup(
        // Set the gyro starting angle based on alliance and assumed robot placement
        new InstantCommand(() -> robotContainer.setGyroAngleToStartMatch(robotInitialOrientation)),

        // Home the arm (should already be homed but this sets the encoder coordinates)
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmHome")),
        new ArmHomeCommand(RobotContainer.m_ArmRotationSubsystem).withTimeout(3.0),

        // Drive out based on drivetrain encoders to align with and face the Amp
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Move1Meter")),
        new DriveRobotCommand(RobotContainer.m_robotDrive, firstMove, false).withTimeout(3.0),

        // Rotate toward the Amp.  It's really away from the amp as the camera is on the back of the robot.
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "TurnCameraTowardAmp")),
        new RotateRobotCommand(RobotContainer.m_robotDrive, targetPose.getRotation().getRadians(), false).withTimeout(3.0),

        // Wait to see apriltag
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "FindAprilTag")),
        new FindAprilTagCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_PoseEstimatorSubsystem, 
          AutoConstants.kRotationSpeed),

        // set the robot drive x,y,theta to match the pose estimator (ie use camera to set x,y,theta)
        new InstantCommand(() -> robotContainer.alignDriveTrainToPoseEstimator()),

        // Use a trajectory to move close to the amp.
        // This is a place holder for the moment.
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "SwerveController")),
        //swerveControllerCommand.withTimeout(3.0).andThen(() -> m_robotDrive.drive(0, 0, 0, true, true)),
        new SwerveToPoseCommand(m_robotDrive, m_PoseEstimatorSubsystem, ampAprilTag),

        // Move to the scoring position
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "GotoScoringPosition")),
        new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, targetPose),

        // Score the note.
        // The StopRobotMotion keeps the swerve drive wheels from moving during the scoring.
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ScoreNote")),
        new ParallelDeadlineGroup(
          new ScoreNoteInAmp(m_ArmRotationSubsystem, m_ShooterSubsystem),
          new StopRobotMotion(m_robotDrive)
        ),

        // Leave the starting box to get more points.
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "LeaveStartBox")),
        new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, finalPose),

        // quiesce the drive and finish.
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Done"))

      );
      addCommands(fullCommandGroup);
    } else {
      // Do what we can given we do not know which alliance we are in.
      // Currently it does nothing but stop the drive train which should already be stopped.
      addCommands(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive));
    }
  }

}
