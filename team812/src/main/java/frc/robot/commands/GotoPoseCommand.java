// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

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

  final double LINEAR_P = 2.0;
  final double LINEAR_I = 0.0;
  final double LINEAR_D = LINEAR_P * 10.0; // NEW 2/1/2024
  final double ANGULAR_P = 0.08;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = ANGULAR_P * 10.0; // NEW 2/1/2024
  final double POSITION_TOLERANCE = Units.inchesToMeters(2.0);
  final double ROTATION_TOLERANCE = Units.degreesToRadians(5.0);  //TODO Tune these tolerances.
  final double MAX_THROTTLE = 1.0; // 0 to 1 is the possible range.

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
    SmartDashboard.putBoolean("GotoPoseOnTarget", false); // We will need to check in execute
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationErrorToTarget;
    Translation2d translationErrorToTargetCorrectedForRotation;
    double rotationError;
    Rotation2d rotationErrorEstimationToDriveTrain;
    Pose2d estimatedPose;
    Pose2d driveTrainPose;
    double estimatedRotationToDriveTrainRotation;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    //SmartDashboard.putNumber("Range", -54);
    estimatedPose = m_PoseEstimatorSubsystem.getCurrentPose();
    SmartDashboard.putString("GotoPose Pose", estimatedPose.toString());
    // Calculate the X and Y and rotation offsets to the target location
    translationErrorToTarget = new Translation2d( m_targetPose.getX() - estimatedPose.getX(), m_targetPose.getY() - estimatedPose.getY());
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    rotationError = m_targetPose.getRotation().getRadians() - estimatedPose.getRotation().getRadians();
    rotationError = rotationError % (2.0*Math.PI); // Could be positive or negative
    if (rotationError > Math.PI)
      rotationError = 2.0 * Math.PI - rotationError;
    else if (rotationError < -Math.PI)
      rotationError = rotationError + 2.0*Math.PI; 
    SmartDashboard.putNumber("GotoPose RError", Units.radiansToDegrees(rotationError));
    SmartDashboard.putString("GotoPoseXYOffset", translationErrorToTarget.toString());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < POSITION_TOLERANCE
    &&  Math.abs(translationErrorToTarget.getY()) < POSITION_TOLERANCE
    &&  Math.abs(rotationError) < ROTATION_TOLERANCE) {
      // Yes, we have arrived
      SmartDashboard.putBoolean("GotoPoseOnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // TODO fine tune PID Controllers and max speeds
      // Rotate the drive X and Y taking into account the difference in the coordinates
      // between the DriveTrain and the PoseEstimator.
      // Calculate the difference in rotation between the PoseEstimator and the DriveTrainPose
      driveTrainPose = m_DriveSubsystemSRXSubsystem.getPose();
      estimatedRotationToDriveTrainRotation = estimatedPose.getRotation().getRadians() - driveTrainPose.getRotation().getRadians();
      estimatedRotationToDriveTrainRotation = estimatedRotationToDriveTrainRotation % (2.0*Math.PI); // Could be positive or negative
      rotationErrorEstimationToDriveTrain = new Rotation2d(estimatedRotationToDriveTrainRotation);
      translationErrorToTargetCorrectedForRotation = translationErrorToTarget.rotateBy(rotationErrorEstimationToDriveTrain);    // TODO Check sign of rotation.
      xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTargetCorrectedForRotation.getX(), 0), -MAX_THROTTLE, MAX_THROTTLE);
      ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTargetCorrectedForRotation.getY(), 0), -MAX_THROTTLE, MAX_THROTTLE);
      /*
       * Next 2 lines I think were breaking the rotation direction calculations to turn in the closest direction so 
       * I commented them out 2/1/2024
       *
       * if (rotationError < 0.0)
       *  rotationError += 2.0*Math.PI; // For the PID Controller make sure the rotationError is between 0 and 2*PI
       */
      rotationSpeed = -MathUtil.clamp(-rotationController.calculate(rotationError, 0),-1.0,1.0); // TODO Check sign  & Clean up 3 negations :-)
    }
    SmartDashboard.putNumber("GotoPose xSpeed", xSpeed);
    SmartDashboard.putNumber("GotoPose ySpeed", ySpeed);
    SmartDashboard.putNumber("GotoPose rSpeed", rotationSpeed);
    // TODO Transform xSpeed/Yspeed based on the difference between the drivetrain X,Y axes and the PoseEstimator X,Y Axes
    m_DriveSubsystemSRXSubsystem.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true); // TODO Verify signs of inputs 

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
