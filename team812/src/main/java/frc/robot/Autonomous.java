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
import frc.robot.commands.CompoundCommands;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.StopRobotMotion;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
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
    m_robotDrive = m_robotContainer.m_robotDrive;
    m_ArmRotationSubsystem = m_robotContainer.m_ArmRotationSubsystem;
    m_ShooterSubsystem = m_robotContainer.m_ShooterSubsystem;
    m_PoseEstimatorSubsystem = m_robotContainer.m_PoseEstimatorSubsystem;
    enum AutonomousStrategy {
      LABTEST,
      BLUEALLIANCE,
      REDALLIANCE,
      USEALLIANCE
    }

    AutonomousStrategy autonomousStrategy = AutonomousStrategy.LABTEST;
    autonomousStrategy = AutonomousStrategy.BLUEALLIANCE;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    if (autonomousStrategy == AutonomousStrategy.LABTEST) {
      // Use the current pose estimator's result for the robots actual pose
      //m_robotDrive.resetOdometry(startingPose);
      m_robotContainer.alignDriveTrainToPoseEstimator();

      // An example trajectory to follow. All units in meters.
      Pose2d startingPose = m_PoseEstimatorSubsystem.getCurrentPose();
      
      //double x=2;
      //double y = 2;
      //double theta = Units.degreesToRadians(-129.0);
      ////x = 1;
      ////y = 0;
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          
          startingPose,
          // Pass through these two interior waypoints, making an 's' curve path
          //List.of(new Translation2d(x+1, y+1), new Translation2d(x+2, y+ -1), new Translation2d(x+3,y+0)),
          List.of(FieldConstants.NearBlueAmp, FieldConstants.NearBandSaw, FieldConstants.NearDriverStation, FieldConstants.NearHammers, FieldConstants.NearNorthDoorToClassroom),
          // End 3 meters straight ahead of where we started, facing forward
          startingPose,
          config);

      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);


      // Run path following command, then stop at the end.
      addCommands(swerveControllerCommand); // .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    } else if ((autonomousStrategy == AutonomousStrategy.USEALLIANCE && Utilities.isBlueAlliance()) || 
               (autonomousStrategy == AutonomousStrategy.BLUEALLIANCE)) {
      // We are in the Blue Alliance.
      //   Drive toward april tag # 6.
      //   Rotate 180 degrees so that we are positioned for shooting.
      //   Back in the last 1 meter to be touching the wall.
      //   Raise the arm to the shooting position.
      //   Run the shooting/outtake motor to score the "note".
      Pose2d targetPose = m_PoseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_AMP.id());
      Rotation2d finalRotation = targetPose.getRotation(); // This will put thte back of the robot towards the april tag.
      Rotation2d nearTagRotation = targetPose.getRotation().rotateBy(new Rotation2d(Math.PI)); // This will face the april tag.
      Pose2d finalPose = new Pose2d(targetPose.getX(), targetPose.getY() - 0.5, finalRotation); // Pose for robot to be at the april tag.
      Pose2d nearTagPose = new Pose2d(targetPose.getX(), targetPose.getY() - 1.0, nearTagRotation);
      Pose2d startingPose = m_PoseEstimatorSubsystem.getCurrentPose();
      int lastAprilTagSeen = m_PoseEstimatorSubsystem.lastAprilTagSeen(); // We can use this to be sure we have the right alliance and have decent field coordinates.
      Utilities.toSmartDashboard("AutoTarget", targetPose);
      Utilities.toSmartDashboard("AutoFinal", finalPose);
      Utilities.toSmartDashboard("AutoNearTag", nearTagPose);
      Utilities.toSmartDashboard("AutoStart", startingPose);
      
      // Use the current pose estimator's result for the robots actual pose
      //m_robotDrive.resetOdometry(startingPose);
      m_robotContainer.alignDriveTrainToPoseEstimator();

      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          
          startingPose,  // We are starting where we are.
          // Pass through these zero interior waypoints, this should probably be something to make sure we dont crash into other robots.
          List.of(),
          nearTagPose,
          config);

      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier  // Should this be the PoseEstimator??
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);
        SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup(
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "SwerveController")),
          swerveControllerCommand.withTimeout(10.0).andThen(() -> m_robotDrive.drive(0, 0, 0, true, true)),
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "GotoPose1")),

          new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, finalPose.getX(), finalPose.getY(), 
            finalPose.getRotation().getRadians()).withTimeout(10.0),
          new ParallelDeadlineGroup(
            new CompoundCommands().ScoreNoteInAmp(m_ArmRotationSubsystem, m_ShooterSubsystem),
            new StopRobotMotion(m_robotDrive)
          ),
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "GotoPose2")),
          new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, 
            startingPose.getX(), startingPose.getY(), startingPose.getRotation().getRadians()).withTimeout(10.0),
          //new WaitCommand(10.0),
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Done"))

        );
        addCommands(fullCommandGroup);

    } else if ((autonomousStrategy == AutonomousStrategy.USEALLIANCE && Utilities.isRedAlliance()) ||
               (autonomousStrategy == AutonomousStrategy.REDALLIANCE)) {
      // Perform the Red Alliance autonomous moves.
      // TODO Figure out what the red alliance strategy is exactly...
    } else {
      // Do what we can given we do not know which alliance we are in.
      // Currently it does nothing but stop the drive train which should already be stopped.
      addCommands(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive));
    }
    addCommands(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)); // Should never reach this code.
  }

}
