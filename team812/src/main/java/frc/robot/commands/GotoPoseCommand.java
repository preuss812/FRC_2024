// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.OIConstants;

public class GotoPoseCommand extends Command {
  /** Creates a new command to move the robot to the specified pose. */
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
  private final DriveSubsystemSRX m_DriveSubsystemSRXSubsystem;
  private final Pose2d m_targetPose;

  // TODO Get these values from Constants.java
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(32.5);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(6.0);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.0);
  final double GOAL_RANGE_METERS = Units.feetToMeters(0.5);

  final double LINEAR_P = 0.0;
  final double LINEAR_D = 0.0;
  final double ANGULAR_P = 0.04;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0;
  final double POSITION_TOLERANCE = Units.inchesToMeters(2.0);
  final double ROTATION_TOLERANCE = Units.degreesToRadians(5.0);  //TODO Tune these tolerances.
  final double MAX_THROTTLE = 0.1; // 0 to 1 is the possible range.

  PIDController xController;
  PIDController yController;
  PIDController rotationController;
  boolean onTarget;

  public GotoPoseCommand(PoseEstimatorSubsystem PoseEstimatorSubsystem
    , DriveSubsystemSRX DriveSubsystemSRXSubsystem
    , double targetX
    , double targetY
    , double targetRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PoseEstimatorSubsystem = PoseEstimatorSubsystem;
    m_DriveSubsystemSRXSubsystem = DriveSubsystemSRXSubsystem;
    m_targetPose = new Pose2d(targetX, targetY, new Rotation2d(targetRotation));
    onTarget = false;
    addRequirements(PoseEstimatorSubsystem, DriveSubsystemSRXSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angular_P = 0.05; // TODO RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, 0.0, 0.1);
    double angular_I = 0.005; // TODO RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotY, 0.0, 0.01);
    SmartDashboard.putNumber("Target angular_P", angular_P);
    SmartDashboard.putNumber("Target angular_I", angular_I);

    xController = new PIDController(LINEAR_P, 0, LINEAR_D);
    yController = new PIDController(LINEAR_P, 0, LINEAR_D);

    rotationController = new PIDController(angular_P, angular_I, ANGULAR_D);
    rotationController.setTolerance(1.0); // did not work, dont understand yet
    onTarget = false;
    SmartDashboard.putBoolean("GotoPoseOnTarget", false); // We will need to check in execute
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d error;
    Pose2d currentPosition;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    //SmartDashboard.putNumber("Range", -54);
    currentPosition = m_PoseEstimatorSubsystem.getCurrentPose();
    error = currentPosition.relativeTo(m_targetPose);
    SmartDashboard.putString("GotoPoseError", error.toString());
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(error.getX()) < POSITION_TOLERANCE
    &&  Math.abs(error.getY()) < POSITION_TOLERANCE
    &&  Math.abs(error.getRotation().getRadians()) < ROTATION_TOLERANCE) {
      // Yes, we have arrived
      SmartDashboard.putBoolean("GotoPoseOnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // TODO fine tune PID Controllers and max speeds
        
      xSpeed = -MathUtil.clamp(xController.calculate(error.getX(), 0), -MAX_THROTTLE, MAX_THROTTLE);
      ySpeed = -MathUtil.clamp(yController.calculate(error.getY(), 0), -MAX_THROTTLE, MAX_THROTTLE);

      // forwardSpeed= 0.0; // for safety in debug do not move forward, just turn.
      rotationSpeed = -MathUtil.clamp(-rotationController.calculate(error.getRotation().getRadians(), 0),-1.0,1.0);
    }
    SmartDashboard.putNumber("GotoPose xSpeed", xSpeed);
    SmartDashboard.putNumber("GotoPose xSpeed", ySpeed);
    SmartDashboard.putNumber("Target Rot speed", rotationSpeed);
    //m_DriveSubsystemSRXSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true, true); // TODO Verify signs of inputs 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget; // Run forever to make debug easier.
    // return true;
  }
}
