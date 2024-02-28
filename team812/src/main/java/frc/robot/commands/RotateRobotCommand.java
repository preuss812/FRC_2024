// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;

/**
 * This comand drives the robot the specified distance
 * If controlRotation is set, it will try to hold the angle in the move pose.
 * If controlRotation is set, it will just manage the X,Y of the move.
 * All angles are in radians.
 */
public class RotateRobotCommand extends Command {
  private final DriveSubsystemSRX robotDrive;
  private final double theta;
  private final boolean relative;
  private double startingTheta;
  private double targetTheta;
  
  private PIDController rotationController;
  private boolean onTarget;

  final double ANGULAR_P = 0.16;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0; // ANGULAR_P * 10.0; // NEW 2/1/2024
  final double ROTATION_TOLERANCE = Units.degreesToRadians(5.0);  //TODO Tune these tolerances.
  final double MAX_THROTTLE = 0.2; // 0 to 1 is the possible range.  // TODO undo Slowed from 1.0 to 0.2

  /** Creates a new DriveDistanceCommand. */
  public RotateRobotCommand(DriveSubsystemSRX robotDrive, double theta, boolean relative) {
    this.robotDrive = robotDrive;
    this.theta = theta;
    this.relative = relative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
;  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // get the robot's current rotation from the drivetrain
    startingTheta = robotDrive.getPose().getRotation().getRadians(); 

    // Calculate the target angle using absolute or relative depending on <relative>.
    // Ensure that the target is between -pi and pi to avoid rotating the long
    // way around.
    if (relative)
      targetTheta = MathUtil.inputModulus(startingTheta + theta, -Math.PI, Math.PI);
    else
      targetTheta =  MathUtil.inputModulus(theta, -Math.PI, Math.PI);
   
    rotationController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    rotationController.setTolerance(1.0); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians). // NEW 2/1/2024
    onTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationError = 0.0;
    double currentTheta;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    currentTheta = robotDrive.getPose().getRotation().getRadians();
    SmartDashboard.putNumber("RR theta", currentTheta);
    SmartDashboard.putNumber("RR target", targetTheta);
    
    rotationError = MathUtil.inputModulus(targetTheta - currentTheta, -Math.PI, Math.PI);
    SmartDashboard.putNumber("RR Error", Units.radiansToDegrees(rotationError));
    
    // Test to see if we have arrived at the requested angle within the specified tolerance.
    if (Math.abs(rotationError) < ROTATION_TOLERANCE) {
      // Yes, we have arrived
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      rotationSpeed = MathUtil.clamp(rotationController.calculate(rotationError, 0),-1.0,1.0);
      onTarget = false;
    }
    SmartDashboard.putBoolean("RR OnTarget", onTarget);
    SmartDashboard.putNumber("RR rSpeed", rotationSpeed);
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
    return onTarget; 
  }
}
