/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;
import frc.robot.commands.ArmHomeCommand;
import frc.robot.commands.ArmRotationCommand;
import frc.robot.commands.DriveRobotCommand;
import frc.robot.commands.FindAprilTagCommand;
import frc.robot.commands.ScoreNoteInAmp;
import frc.robot.commands.GotoAmpCommand;
//import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.RotateRobotCommand;
//import frc.robot.commands.StopRobotMotion;
//import frc.robot.commands.PushTowardsWall;
import frc.robot.commands.PushTowardsWallUltrasonic;
import frc.robot.Constants.ArmConstants;
//import frc.robot.commands.SwerveToPoseCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.FieldConstants;
//import frc.robot.Constants.VisionConstants;
//import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.Constants.FieldConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  private final DriveSubsystemSRX m_robotDrive;
  private final ArmRotationSubsystem m_ArmRotationSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final PingResponseUltrasonicSubsystem m_PingResponseUltrasonicSubsystem;
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;

  public Autonomous(RobotContainer robotContainer) {

    m_robotDrive = RobotContainer.m_robotDrive;
    m_ArmRotationSubsystem = RobotContainer.m_ArmRotationSubsystem;
    m_ShooterSubsystem = RobotContainer.m_ShooterSubsystem;
    m_PoseEstimatorSubsystem = RobotContainer.m_PoseEstimatorSubsystem;
    m_PingResponseUltrasonicSubsystem = RobotContainer.m_PingResponseUltrasonicSubsystem;

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
      final double firstMoveX = 1.84 - DriveConstants.kBackToCenterDistance;
      final double firstMoveY = 0.0;
      final double finalMoveX = 2.0; // Arbitrary move out of the starting box
      final double finalMoveY = -1.0; // Arbitrary move out of the starting box and away from the wall.
      
      Pose2d finalMove;
      Pose2d firstMove;
      finalMove = new Pose2d(finalMoveX, finalMoveY, new Rotation2d( Math.PI/2.0)); // Pose for robot to face the center of the field.
      firstMove = new Pose2d(firstMoveX, firstMoveY, new Rotation2d(-Math.PI/2.0)); // Pose for robot to be at the april tag.

      SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup(
        // Set the gyro starting angle based on alliance and assumed robot placement
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 1)),
        new InstantCommand(() -> robotContainer.setGyroAngleToStartMatch()),
        new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.SPEED)),
        new InstantCommand(() -> Utilities.allianceSetCurrentPose(
          new Pose2d(
            DriveConstants.kBackToCenterDistance,
            DriveConstants.kApproximateStartingY,
            new Rotation2d(DriveConstants.kStartingOrientation)))),

        // Home the arm (should already be homed but this sets the encoder coordinates)
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 2)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmHome")),
        new ArmHomeCommand(RobotContainer.m_ArmRotationSubsystem).withTimeout(3.0),

        // Drive out based on drivetrain encoders to align with and face the Amp
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 3)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Move1Meter")),
        new DriveRobotCommand(RobotContainer.m_robotDrive, firstMove, false).withTimeout(5.0), // TODO Try controlRotation == true.

        // Rotate toward the Amp.  It's really away from the amp as the camera is on the back of the robot.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 4)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "TurnCameraTowardAmp")),
        new RotateRobotCommand(RobotContainer.m_robotDrive, -Math.PI/2, false).withTimeout(5.0),

        // Wait to see apriltag
        //new InstantCommand(() -> Utilities.refineYCoordinate()),  // TODO test this and see if we can reduce or eliminate the wait.
        new WaitCommand(2.5), // TODO reduce or eliminate wait.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 5)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "FindAprilTag")),
        new FindAprilTagCommand(
          RobotContainer.m_robotDrive,
          RobotContainer.m_PoseEstimatorSubsystem, 
          AutoConstants.kRotationSpeed).withTimeout(10.0), // This is too slow for just 10 seconds

        // set the robot drive x,y,theta to match the pose estimator (ie use camera to set x,y,theta)
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 6)),
        new InstantCommand(() -> robotContainer.alignDriveTrainToPoseEstimator()),

        // Use a trajectory to move close to the amp.
        // This is a place holder for the moment.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 7)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "SwerveController")),
        //new SwerveToPoseCommand(m_robotDrive, m_PoseEstimatorSubsystem, ampAprilTag),

        // Move to the scoring position
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 8)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "GotoScoringPosition")),
        new ParallelCommandGroup(
          new GotoAmpCommand(m_PoseEstimatorSubsystem, m_robotDrive).withTimeout(3.0),
          
         new ArmRotationCommand(m_ArmRotationSubsystem, ArmConstants.kArmMinPosition)), // TODO raise arm in parallel. 100 fudge factor
        // TODO: Could try raising the arm in parallel with this move to the amp - dph 2024-03-06.

        // Score the note.
        // The StopRobotMotion keeps the swerve drive wheels from moving during the scoring.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 9)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ScoreNote")),
        new ParallelDeadlineGroup(
          new ScoreNoteInAmp(m_ArmRotationSubsystem, m_ShooterSubsystem),
          new PushTowardsWallUltrasonic(m_robotDrive, m_PingResponseUltrasonicSubsystem)
        ).withTimeout(10.0),

        // Leave the starting box to get more points.
        //new InstantCommand(() -> Utilities.refineYCoordinate()),  // TODO test this and see if we can reduce or eliminate the wait.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 10)),
        new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "LeaveStartBox")),
        new DriveRobotCommand(m_robotDrive, finalMove, true).withTimeout(5.0),

        // quiesce the drive and finish.
        new InstantCommand(() -> SmartDashboard.putNumber("Auto Step", 0)),
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
