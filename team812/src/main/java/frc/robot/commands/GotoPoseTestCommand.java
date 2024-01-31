// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GotoPoseTestCommand extends InstantCommand {
  public GotoPoseTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    double POSITION_TOLERANCE = Units.inchesToMeters(2.0);
    double ROTATION_TOLERANCE = Units.degreesToRadians(5.0);

    Translation2d translationErrorToTarget;
    Translation2d translationErrorToTargetCorrectedForRotation;
    double rotationError;
    Rotation2d rotationErrorEstimationToDriveTrain;
    Pose2d estimatedPose;
    Pose2d driveTrainPose;
    Pose2d m_targetPose;
    double estimatedRotationToDriveTrainRotation;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    boolean onTarget = false;
    
    double rotationSpeed = 0.0;
    //SmartDashboard.putNumber("Range", -54);
    estimatedPose = new Pose2d(1.0, 5.0,new Rotation2d(0.0));
    m_targetPose = new Pose2d(1.84, 7.2, new Rotation2d(Math.PI/2.0));
    
    // Calculate the X and Y and rotation offsets to the target location
    translationErrorToTarget = new Translation2d(m_targetPose.getX() - estimatedPose.getX(), m_targetPose.getY() - estimatedPose.getY());
    SmartDashboard.putNumber("GotoPose X", translationErrorToTarget.getX());
    SmartDashboard.putNumber("GotoPose Y", translationErrorToTarget.getY());
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    rotationError = m_targetPose.getRotation().getRadians() - estimatedPose.getRotation().getRadians();
    SmartDashboard.putNumber("GotoPose Theta1", Units.radiansToDegrees(rotationError));
    rotationError = rotationError % (2.0*Math.PI); // Could be positive or negative
    if (rotationError > Math.PI)
      rotationError = 2.0 * Math.PI - rotationError;
    else if (rotationError < -Math.PI)
      rotationError = rotationError + 2.0*Math.PI;
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
      driveTrainPose = estimatedPose.rotateBy(new Rotation2d(Units.degreesToRadians(15.0)));
      estimatedRotationToDriveTrainRotation = estimatedPose.getRotation().getRadians() - driveTrainPose.getRotation().getRadians();
      estimatedRotationToDriveTrainRotation = estimatedRotationToDriveTrainRotation % (2.0*Math.PI); // Could be positive or negative
      SmartDashboard.putNumber("GotoPose Theta", Units.radiansToDegrees(rotationError));
      SmartDashboard.putNumber("GotoPose ThetaCorrection", Units.radiansToDegrees(estimatedRotationToDriveTrainRotation));
      rotationErrorEstimationToDriveTrain = new Rotation2d(estimatedRotationToDriveTrainRotation);
      translationErrorToTargetCorrectedForRotation = translationErrorToTarget.rotateBy(rotationErrorEstimationToDriveTrain);    // TODO Check sign of rotation.
      SmartDashboard.putNumber("GotoPose X'", translationErrorToTargetCorrectedForRotation.getX());
      SmartDashboard.putNumber("GotoPose Y'", translationErrorToTargetCorrectedForRotation.getY());
      //xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTargetCorrectedForRotation.getX(), 0), -MAX_THROTTLE, MAX_THROTTLE);
      //ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTargetCorrectedForRotation.getY(), 0), -MAX_THROTTLE, MAX_THROTTLE);

      if (rotationError < 0.0)
        rotationError += 2.0*Math.PI; // For the PID Controller make sure the rotationError is between 0 and 2*PI
      //rotationSpeed = -MathUtil.clamp(-rotationController.calculate(rotationError, 0),-1.0,1.0);
    }
    //SmartDashboard.putNumber("GotoPose xSpeed", xSpeed);
    //SmartDashboard.putNumber("GotoPose ySpeed", ySpeed);
    //SmartDashboard.putNumber("GotoPose rSpeed", rotationSpeed);
    // TODO Transform xSpeed/Yspeed based on the difference between the drivetrain X,Y axes and the PoseEstimator X,Y Axes
    //m_DriveSubsystemSRXSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true, true); // TODO Verify signs of inputs 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
