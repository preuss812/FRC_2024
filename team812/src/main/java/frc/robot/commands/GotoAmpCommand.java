// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This command drives the robot to the AMP for the robot's alliance.
 * It will eventually follow a pre-planned trajectory from anywhere on the
 * field to the AMP.  It should probably lower the arm and quiese the intake
 * shooter motors as well.
 * The anywhere on the field part is not implemented here yet as it has some issues.
 */
public class GotoAmpCommand extends GotoPoseCommand {
  /** Creates a new GotoAmpCommand. */
  public GotoAmpCommand(PoseEstimatorSubsystem PoseEstimatorSubsystem
    , DriveSubsystemSRX DriveSubsystemSRXSubsystem) { 
    super(PoseEstimatorSubsystem,DriveSubsystemSRXSubsystem, new Pose2d()); // Pose is a place holder.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //double angular_P = 0.05; // TODO RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, 0.0, 0.1);  // Removed 2/1/2024
    //double angular_I = 0.005; // TODO RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotY, 0.0, 0.01); // Removed 2/1/2024
    //SmartDashboard.putNumber("Target angular_P", angular_P);
    //SmartDashboard.putNumber("Target angular_I", angular_I);

    xController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
    xController.setIZone(0.1); // This is meters so about 4 inches  // TODO Needs tuning.
    yController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
    yController.setIZone(0.1); // NEW 2/1/2024 // TODO Needs Tuning.

    rotationController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    rotationController.setTolerance(1.0); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians). // NEW 2/1/2024
    onTarget = false;

    if (Utilities.isBlueAlliance()) {
      Pose2d tag = m_PoseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.BLUE_AMP.id());
      targetPose = new Pose2d(tag.getX(), tag.getY() - 0.4, tag.getRotation());

    } else if (Utilities.isRedAlliance()) {
      targetPose = m_PoseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.RED_AMP.id());
    }
    else {
      targetPose = m_PoseEstimatorSubsystem.getCurrentPose(); // Hack:: if we dont know the alliance. Dont move. 
    }

    Utilities.toSmartDashboard("GotoTarget", targetPose);
    SmartDashboard.putBoolean("GotoPoseOnTarget", false); // We will need to check in execute
  }
}
