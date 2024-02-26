// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class FollowTrajectoryCommand extends SequentialCommandGroup {
  
  // These trajectory config parameters are assumed to be the same for all trajectories.
  // If that is not true, this would need to be moved elsewere or cloned+mutated to provide options.
  public static final TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  /** Creates a new FollowTrajectoryCommand. */
  public FollowTrajectoryCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    TrajectoryConfig config,
    Pose2d startingPose,
    List<Translation2d> waypoints,
    Pose2d targetPose) 
  {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(          
          startingPose,  // We are starting where we are.
          // Pass through these zero interior waypoints, this should probably be something to make sure we dont crash into other robots.
          waypoints,
          targetPose,
          config);
    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      robotDrive::setModuleStates,
      robotDrive);
    
    addCommands(swerveControllerCommand);
  }

}
