// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.Utilities;

public class SwerveToPoseCommand extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final AprilTag destination;
  private SequentialCommandGroup commands;

  /** Creates a new SwerveToPoseCommand. */
  public SwerveToPoseCommand(
    DriveSubsystemSRX robotDrive,
    PoseEstimatorSubsystem poseEstimatorSubsystem,
    AprilTag destination
  ) {
    this.robotDrive = robotDrive;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.destination = destination;

    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(robotDrive, poseEstimatorSubsystem);  // This may be a problem due to self cancellation.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose = poseEstimatorSubsystem.getCurrentPose();
    Utilities.toSmartDashboard("SW Start",startingPose);
    SmartDashboard.putString("SW","init");

    List<Translation2d> waypoints = new ArrayList<>();
    Pose2d aprilTagPose = null;
    Pose2d nearTargetPose = null;
    Pose2d targetPose = null;
    SmartDashboard.putString("SW","Running");
    commands = new SequentialCommandGroup();

    if (destination == AprilTag.BLUE_AMP && Utilities.isBlueAlliance()) {
      waypoints = Utilities.planBlueAmpTrajectory(startingPose);
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_AMP.id());
      //nearTargetPose = Utilities.backToPose(aprilTagPose, 1.5);
      targetPose = Utilities.backToPose(aprilTagPose, 0.5);
    } else if (destination == AprilTag.RED_AMP && Utilities.isRedAlliance()) {
    } else if (destination == AprilTag.BLUE_RIGHT_SOURCE && Utilities.isBlueAlliance()) {
    } else if (destination == AprilTag.RED_LEFT_SOURCE && Utilities.isRedAlliance()) {
    } else {
      targetPose = startingPose; // This will end up doing nothing.
      nearTargetPose = startingPose;
    }
    if (waypoints.size() > 0)
      SmartDashboard.putString("SW plan", waypoints.toString());
      //Utilities.toSmartDashboard("SW target", nearTargetPose);
    if (startingPose != null && waypoints.size() > 0 && nearTargetPose != null)
      commands.addCommands(new FollowTrajectoryCommand(robotDrive, poseEstimatorSubsystem, null, startingPose, waypoints, nearTargetPose));
    
    if (false && targetPose != null) 
      commands.addCommands(
        new InstantCommand(() -> SmartDashboard.putString("SW", "GotoPose")),
        new GotoPoseCommand(poseEstimatorSubsystem, robotDrive, targetPose
        ));
    commands.schedule();  // This will release the command which is not what we want.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      commands.cancel();
    } catch (Exception e) {
    }
    SmartDashboard.putBoolean("SW Interrupted", interrupted);
    SmartDashboard.putString("SW","Done abort");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = true;
    try {
      finished = commands.isFinished();
    } catch (Exception e) {
      finished = true;
    }
    return finished;
  }
}
