// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.photonvision.PhotonCamera;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;  // To access the Black Box Controller.
import frc.robot.Utilities;

public class DriveOnAprilTagProjectionCommand extends Command {
  
  public class DriveOnAprilTagProjectionConfig {
    private double maxThrottle;
    private double linearP;
    private double linearI;
    private double linearD;
    private double linearF;
    private double linearIZone;
    private double linearTolerance;

    private double maxRotation;
    private double angularP;
    private double angularI;
    private double angularD;
    private double angularF;
    private double angularIZone;
    private double angularTolerance;

    /**
     * default constructor
     */
    public DriveOnAprilTagProjectionConfig() {
      maxThrottle = 0.70;
      linearP = 2.0;
      linearI = 0.0; // linearP/100.0;
      linearD = 0.0; // linearP*10.0;
      linearF = 0.0;
      linearIZone = Units.inchesToMeters(4.0);
      linearTolerance = Units.inchesToMeters(2.0);

      maxRotation = 0.8;
      angularP =  0.35;
      angularI = 0.0; // angularI/100.0;
      angularD = 0.0; //angularP*10.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(10.0);
      angularTolerance = Units.degreesToRadians(5.0);
    }
    public DriveOnAprilTagProjectionConfig setMaxThrottle(double maxThrottle) {this.maxThrottle = maxThrottle; return this; };
    public DriveOnAprilTagProjectionConfig setLinearP(double linearP) {this.linearP = linearP; return this; };
    public DriveOnAprilTagProjectionConfig setLinearI(double linearI) {this.linearI = linearI; return this; };
    public DriveOnAprilTagProjectionConfig setLinearD(double linearD) {this.linearD = linearD; return this; };
    public DriveOnAprilTagProjectionConfig setLinearF(double linearF) {this.linearF = linearF; return this; };
    public DriveOnAprilTagProjectionConfig setLinearIZone(double linearIZone) {this.linearIZone = linearIZone; return this; };
    public DriveOnAprilTagProjectionConfig setLinearTolerance(double linearTolerance) {this.linearTolerance = linearTolerance; return this; };

    public DriveOnAprilTagProjectionConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public DriveOnAprilTagProjectionConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public DriveOnAprilTagProjectionConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public DriveOnAprilTagProjectionConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public DriveOnAprilTagProjectionConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public DriveOnAprilTagProjectionConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public DriveOnAprilTagProjectionConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };

    public double getMaxThrottle() { return maxThrottle; }
    public double getLinearP() { return linearP; }
    public double getLinearI() { return linearI; }
    public double getLinearD() { return linearD; }
    public double getLinearF() { return linearF; }
    public double getLinearIZone() { return linearIZone; }
    public double getLinearTolerance() { return linearTolerance; }
    
    public double getMaxRotation() { return maxRotation; }
    public double getAngularP() { return angularP; }
    public double getAngularI() { return angularI; }
    public double getAngularD() { return angularD; }
    public double getAngularF() { return angularF; }
    public double getAngularIZone() { return angularIZone; }
    public double getAngularTolerance() { return angularTolerance; }
  } // DriveOnAprilTagProjectionConfig Class

  /** 
   * Creates a new command to move the robot along the line that projects perpendicularly from an april tag.
   */
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DriveSubsystemSRX robotDrive;
  private final PhotonCamera photonCamera;
  private final XboxController xbox;
  private final DriveOnAprilTagProjectionConfig config;

  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private Pose2d targetPose;
  private Pose2d tagPose;
  
  private double p; // The constant P from the normal form for the april tag projection line.
  private double xSign; // The sign of x axis motion toward the april tag.
  private double ySign; // The sign of y axis motion toward the april tag. 
  private double projectionM;
  private double projectionB;

  private boolean onTarget;
  private boolean debug = false;
  private final int debugMinIterations = 5*50; // For debug do not end the command so we can observe oscillations.
  private int debugIterations = 0;

  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params photonCamera - the Photon Camera Subsystem used to see the April Tags.
   */
  public DriveOnAprilTagProjectionCommand(PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , XboxController xbox) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.photonCamera = photonCamera;
    this.xbox = xbox;
    onTarget = false;
    this.config = new DriveOnAprilTagProjectionConfig();
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  public DriveOnAprilTagProjectionCommand(PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , XboxController xbox
    , DriveOnAprilTagProjectionConfig config) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.photonCamera = photonCamera;
    this.xbox = xbox;
    onTarget = false;
    this.config = config;
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = true; // Not really but if we dont find a target, this will cause the command to end immediately.
    p = 0.0;
    xSign = 0.0;
    ySign = 0.0;
    debug = RobotContainer.m_BlackBox.isSwitchCenter();
    double linearP = 0; // If no tag is found, do not energize the motors.
    double linearI = 0;

    if (debug) {
      debugIterations = 0;
      config.setLinearTolerance(0.01); // tighter tolerance of 1cm
      linearP = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, 0.0, 5.0);
      linearI = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotY, 0.0, 0.1);
      
    }
    var pipelineResult = photonCamera.getLatestResult();
    if (pipelineResult.hasTargets()) {
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      SmartDashboard.putNumber("DA FID", fiducialId);

      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      tagPose = poseEstimatorSubsystem.getAprilTagPose(fiducialId);
      // Re-align the robot drive and pose subsystem.
      robotDrive.resetOdometry(poseEstimatorSubsystem.getCurrentPose());

        onTarget = false;
        if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0) {
          linearP = config.getLinearP();
          linearI = config.getLinearI();
          Utilities.toSmartDashboard("DA tag",tagPose);
  
        // Compute the projected line from the april tag.
        // I'm using the "normal form" of the line.
        double theta = tagPose.getRotation().getRadians();
        SmartDashboard.putNumber("DP theta", theta);
        SmartDashboard.putNumber("DP cos", Math.cos(theta));

        p = tagPose.getX()* Math.cos(theta) + tagPose.getY() * Math.sin(theta);
        SmartDashboard.putNumber("DP p", p);

        // We also need to determine which direction we are moving toward the line.
        // In other words, which way is towards the target along the line.
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        final double epsilon = 0.0001;
        if (Math.abs(cos) < epsilon) cos = 0.0;
        if (Math.abs(sin) < epsilon) sin = 0.0;
        xSign = -Math.signum(cos);
        ySign = -Math.signum(sin);
        if (xSign !=0 ) {
          projectionM = Math.tan(theta);
          // y = mx+b => y - mx = b;
          projectionB = tagPose.getY() - projectionM * tagPose.getX();
        }
        SmartDashboard.putNumber("DP x", xSign);
        SmartDashboard.putNumber("DP y", ySign);
      }
    }
    SmartDashboard.putNumber("DA P", linearP);
    SmartDashboard.putNumber("DA I", linearI);
    // Create PID controllers for x, y, rotation.
    xController = new PIDController(
      linearP, // config.getLinearP(),
      linearI, // config.getLinearI(),
      config.getLinearD()
    );
    xController.setIZone(config.getLinearIZone());
    yController = new PIDController(
      linearP, // config.getLinearP(),
      linearI, // config.getLinearI(),
      config.getLinearD()
    );
    yController.setIZone(config.getAngularIZone()); // TODO Needs Tuning.

    rotationController = new PIDController(
      config.getAngularP(),
      config.getAngularI(),
      config.getAngularD()
      );
    rotationController.setTolerance(config.getAngularTolerance()); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians).
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationErrorToTarget;
    Translation2d translationErrorToTargetCorrectedForRotation;
    double rotationError;
    Rotation2d rotationErrorEstimationToDriveTrain;
    Pose2d currentPose;
    Pose2d driveTrainPose;
    double estimatedRotationToDriveTrainRotation;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    Translation2d nominalMove;
    Translation2d move;
    Translation2d uncorrectedGoal;
    Translation2d correctedGoal;
    double throttle;

    debugIterations++;
    throttle = -xbox.getRightY();
    throttle = 0.5;
    currentPose = poseEstimatorSubsystem.getCurrentPose();
    nominalMove = new Translation2d(throttle*config.getLinearP()/1.0, 0.0);
    move = nominalMove.rotateBy(tagPose.getRotation().plus(new Rotation2d(Math.PI))); // Rotate the nominal move to the tag pose.
    uncorrectedGoal = currentPose.getTranslation().plus(move);
    SmartDashboard.putString("DA Move",move.toString());
    SmartDashboard.putString("DA unc",uncorrectedGoal.toString());
    
    // We now have the goal position ignoring the distance from the april tag projection
    // Compute a correction factor to keep us on the projection line.
    if (xSign == 0) {
      // The line is vertical.
      // The correction is simple set the X goal to be the X coordinate of the goal line.
      correctedGoal = new Translation2d( tagPose.getX(), uncorrectedGoal.getY());
    } else if (ySign == 0) {
      // The line is horizontal
      // The correction is simple set the Y goal to be the Y coordinate of the goal line.
      correctedGoal = new Translation2d(uncorrectedGoal.getX(), tagPose.getY());
    } else {
      // Intersect the 2 lines to find the point to be on the projection line.
      double perpendicularM = 1/projectionM;
      double perpendicularB = uncorrectedGoal.getY() - projectionM * uncorrectedGoal.getX();
      double intersectX = (perpendicularB - projectionB)/(projectionM - perpendicularM);
      double intersectY = projectionM*intersectX + projectionB;
      correctedGoal = new Translation2d(intersectX, intersectY);
    }

    SmartDashboard.putString("DA goal", correctedGoal.toString());

    // Calculate the X and Y and rotation offsets to the target location
    translationErrorToTarget = new Translation2d( correctedGoal.getX() - currentPose.getX(), correctedGoal.getY() - currentPose.getY());
    
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    rotationError = MathUtil.inputModulus(tagPose.getRotation().getRadians() - currentPose.getRotation().getRadians(), -Math.PI, Math.PI);
    SmartDashboard.putNumber("DA R", Units.radiansToDegrees(rotationError));
    SmartDashboard.putNumber("DA X", translationErrorToTarget.getX());
    SmartDashboard.putNumber("DA Y", translationErrorToTarget.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationErrorToTarget.getY()) < config.getLinearTolerance()
    &&  Math.abs(rotationError) < config.getAngularTolerance()) {
      // Yes, we have arrived
      SmartDashboard.putBoolean("DA OnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // TODO fine tune PID Controllers and max speeds
      // Rotate the drive X and Y taking into account the difference in the coordinates
      // between the DriveTrain and the PoseEstimator.
      // Calculate the difference in rotation between the PoseEstimator and the DriveTrainPose
      driveTrainPose = robotDrive.getPose();
      estimatedRotationToDriveTrainRotation = currentPose.getRotation().getRadians() - driveTrainPose.getRotation().getRadians();
      estimatedRotationToDriveTrainRotation = MathUtil.inputModulus(estimatedRotationToDriveTrainRotation, 0.0, Math.PI*2.0);
      
      SmartDashboard.putNumber("DA P2Derr", Units.radiansToDegrees(estimatedRotationToDriveTrainRotation));

      rotationErrorEstimationToDriveTrain = new Rotation2d(estimatedRotationToDriveTrainRotation);
      translationErrorToTargetCorrectedForRotation = translationErrorToTarget.rotateBy(rotationErrorEstimationToDriveTrain);    // TODO Check sign of rotation.
      xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTargetCorrectedForRotation.getX(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
      ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTargetCorrectedForRotation.getY(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
      /*
       * Next 2 lines I think were breaking the rotation direction calculations to turn in the closest direction so 
       * I commented them out 2/1/2024
       *
       * if (rotationError < 0.0)
       *  rotationError += 2.0*Math.PI; // For the PID Controller make sure the rotationError is between 0 and 2*PI
       */
      rotationSpeed = -MathUtil.clamp(-rotationController.calculate(rotationError, 0),-config.getMaxRotation(), config.getMaxRotation()); // TODO Check sign  & Clean up 3 negations :-)
    }
    rotationSpeed = MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband); // Based on xbox right joystick / ignore calculations above.
    SmartDashboard.putNumber("DA xSpeed", xSpeed);
    SmartDashboard.putNumber("DA ySpeed", ySpeed);
    SmartDashboard.putNumber("DA rSpeed", rotationSpeed);
    robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true); // TODO Verify signs of inputs 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // TODO remove for debug.
    /*if (!debug || (debugIterations >= debugMinIterations))
      return onTarget;
    else
      return false;
    */ 
  }
} // DriveOnAprilTagProjectionCommand class
