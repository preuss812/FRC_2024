// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CameraVisionSubsystem;

import java.util.List;

// import org.photonvision.PhotonUtils;
// import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;  // To access the Black Box Controller.

public class GotoAprilTag extends Command {
  /** Creates a new FollowApriltagCommandBB. */
  private final CameraVisionSubsystem m_cameraSubsystem;
  private final DriveSubsystemSRX m_DriveSubsystemSRX;

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

  PIDController forwardController;
  PIDController turnController;

  double forwardSpeed;
  double rotationSpeed;

  public GotoAprilTag(CameraVisionSubsystem cameraVisionSubsystem
    , DriveSubsystemSRX driveSubsystemSRX
    , Integer apriltagInteger
    , double desiredXOffset
    , double desiredYOffset
    , double desiredPoseAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cameraSubsystem = cameraVisionSubsystem;
    m_DriveSubsystemSRX = driveSubsystemSRX;
    addRequirements(cameraVisionSubsystem, driveSubsystemSRX);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angular_P = 0.05; // TODO RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, 0.0, 0.1);
    double angular_I = 0.005; // TODO RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotY, 0.0, 0.01);
    SmartDashboard.putNumber("Target angular_P", angular_P);
    SmartDashboard.putNumber("Target angular_I", angular_I);

    forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    turnController = new PIDController(angular_P, angular_I, ANGULAR_D);
    turnController.setTolerance(1.0); // did not work, dont understand yet
    forwardSpeed = 0.01; // TODO Pick better speeds
    rotationSpeed = 0.01;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error;
    PhotonTrackedTarget target;
    //SmartDashboard.putNumber("Range", -54);
    target = m_cameraSubsystem.getBestTarget(); // TODO Get the specific April Tag requested
  
    // Query the latest result from PhotonVision
    var result = m_cameraSubsystem.camera.getLatestResult();
    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets(); // TODO Handle empty list of targets

    // If no apriltag found, stop.
    if (target == null) {
        SmartDashboard.putString("TargetFound", "No");
        forwardSpeed= 0.0;
        rotationSpeed = 0.0;
} else {
        SmartDashboard.putString("TargetFound", "Yes");

        double yaw = target.getYaw();
        Transform3d targetTransform = target.getBestCameraToTarget();
        double x = targetTransform.getX();
        double y = targetTransform.getY();
        SmartDashboard.putNumber("TargetX", x);
        SmartDashboard.putNumber("TargetY", y);
        SmartDashboard.putNumber("TargetYaw", yaw);

        double range = Math.sqrt(x*x+y*y);
        SmartDashboard.putNumber("TargetRange", range);
        SmartDashboard.putNumber("TargetRangeGoal", GOAL_RANGE_METERS);

        forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
        // forwardSpeed= 0.0; // for safety in debug do not move forward, just turn.
        rotationSpeed = -MathUtil.clamp(-turnController.calculate(yaw, 0),-1.0,1.0);
        if( Math.abs(rotationSpeed) <= 0.5) {
         // rotationSpeed = 0.0; // so that we don't use battery power for no motion // Commented out for characterization purposes.
        }
        error = turnController.getPositionError();
        SmartDashboard.putNumber("FABB: Turn error", error);
    }
    SmartDashboard.putNumber("Target Fwd speed", forwardSpeed);
    SmartDashboard.putNumber("Target Rot speed", rotationSpeed);
    forwardSpeed = 0.0;
    // TODO: m_DriveSubsystemSRX.preussDrive(forwardSpeed, rotationSpeed); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO Finish when we are close enough to the target
//    return turnController.atSetpoint();
    return false; // Run forever to make debug easier.
    // return true;
  }
}
