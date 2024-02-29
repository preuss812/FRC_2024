// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;

public class DriveRobotCommand extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final Pose2d relativeMove;
  private Pose2d startingPose;
  private Pose2d targetPose;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private boolean onTarget;
  private static int timesInitialized = 0;

  final double LINEAR_P = 2.7;
  final double LINEAR_I = 0.0;
  final double LINEAR_D = 0.0; // LINEAR_P * 10.0; // NEW 2/1/2024
  final double ANGULAR_P = 0.16;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0; // ANGULAR_P * 10.0; // NEW 2/1/2024
  final double POSITION_TOLERANCE = Units.inchesToMeters(2.0);
  final double ROTATION_TOLERANCE = Units.degreesToRadians(5.0);  //TODO Tune these tolerances.
  final double MAX_THROTTLE = 1.0; // 0 to 1 is the possible range.  // Slowed from 1.0 to 0.2

  /** Creates a new DriveDistanceCommand. */
  public DriveRobotCommand(DriveSubsystemSRX robotDrive, Pose2d relativeMove) {
    this.robotDrive = robotDrive;
    this.relativeMove = relativeMove;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
;  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timesInitialized++;
    SmartDashboard.putNumber("DR #inits", timesInitialized);
    // get the robot's current pose from the drivetrain
    startingPose = robotDrive.getPose();
    // add the relativeMove to the startingPose // There is an alliance component to this.   
    // I'm assuming the caller has handled it in the relativeMove.
    targetPose = new Pose2d(
      startingPose.getX() + relativeMove.getX(),
      startingPose.getY() + relativeMove.getY(),
      startingPose.getRotation().rotateBy(relativeMove.getRotation()));
    xController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
    xController.setIZone(0.1); // This is meters so about 4 inches  // TODO Needs tuning.
    yController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
    yController.setIZone(0.1); // NEW 2/1/2024 // TODO Needs Tuning.
    rotationController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    rotationController.setTolerance(1.0); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians). // NEW 2/1/2024
    onTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationError;
    double rotationError;
    Pose2d currentPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    //SmartDashboard.putNumber("Range", -54);
    currentPose = robotDrive.getPose();
    Utilities.toSmartDashboard("Drive Pose", currentPose);
    Utilities.toSmartDashboard("Drive target", targetPose);
    // Calculate the X and Y and rotation offsets to the target location
    translationError = new Translation2d( targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    rotationError = MathUtil.inputModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians(), -Math.PI, Math.PI);
    SmartDashboard.putNumber("Drive R Error", Units.radiansToDegrees(rotationError));
    SmartDashboard.putNumber("Drive X Error", translationError.getX());
    SmartDashboard.putNumber("Drive Y Error", translationError.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationError.getX()) < POSITION_TOLERANCE
    &&  Math.abs(translationError.getY()) < POSITION_TOLERANCE
    &&  Math.abs(rotationError) < ROTATION_TOLERANCE) {
      // Yes, we have arrived
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // TODO fine tune PID Controllers and max speeds      
      xSpeed = MathUtil.clamp(xController.calculate(translationError.getX(), 0), -MAX_THROTTLE, MAX_THROTTLE);
      ySpeed = MathUtil.clamp(yController.calculate(translationError.getY(), 0), -MAX_THROTTLE, MAX_THROTTLE);
      rotationSpeed = -MathUtil.clamp(-rotationController.calculate(rotationError, 0),-1.0,1.0); // TODO Check sign  & Clean up 3 negations :-)
      onTarget = false;
     }
//          rotationSpeed = 0.0;
      SmartDashboard.putBoolean("Drive OnTarget", onTarget);

    SmartDashboard.putNumber("Drive xSpeed", xSpeed);
    SmartDashboard.putNumber("Drive ySpeed", ySpeed);
    SmartDashboard.putNumber("Drive rSpeed", rotationSpeed);
    robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return true;
    return onTarget; 
  }
}
