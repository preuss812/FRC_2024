/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.ArmConstants;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.NoteIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.BlackBoxSubsystem;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.EncoderSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.CameraVisionSubsystem;
import frc.robot.commands.ArmRotationCommand;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.GotoPoseTestCommand;
import frc.robot.commands.NoteIntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.StopRobotMotion;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.CompoundCommands;


import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  //private final DriveTrain m_DriveTrain = new DriveTrain();
  // The robot's subsystems
  private final static DriveSubsystemSRX m_robotDrive = new DriveSubsystemSRX();

 // public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static CameraVisionSubsystem m_CameraVisionSubsystem = new CameraVisionSubsystem();
  public static EncoderSubsystem m_EncoderSubsystem = new EncoderSubsystem();
  public static PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem( m_CameraVisionSubsystem.camera, m_robotDrive);
  public static ArmRotationSubsystem m_ArmRotationSubsystem = new ArmRotationSubsystem();
  public static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public static NoteIntakeSubsystem m_NoteIntakeSubsystem = new NoteIntakeSubsystem();
  public static WinchSubsystem m_WinchSubsystem = new WinchSubsystem();

  //public static DigitalIOSubsystem m_DigitalIOSubsystem = new DigitalIOSubsystem();

  // Controller definitions
  //private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort); // TODO find the right port - dph
  
  public static CANcoder m_enctest = new CANcoder(38);

  // private final Joystick xboxController = new
  // Joystick(OIConstants.kXboxController);
  double POV_to_double(int pov) {
    double result;
    if (pov == -1) {
      result = 0.0;
    } else if (pov == 0) {
      result = 0.5;
    } else if (pov == 180) {
      result = -0.5;
    } else {
      result = 0.0;
    }
    // SmartDashboard.putNumber("POV", pov);
    // SmartDashboard.putNumber("POV_Out", result);
    return result;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

   

    // Configure the button bindings
    configureButtonBindings();

    // for debug: put subsystem info on the shuffleboard. - remove before competition.
    /*
    SmartDashboard.putData(m_ArmExtensionSubsystem);
    SmartDashboard.putData(m_ArmRotationSubsystem);
    SmartDashboard.putData("ArmRotate", new ArmCommand(m_ArmRotationSubsystem, 1000));
    SmartDashboard.putData("ArmHomeCommand", new ArmHomeCommand(m_ArmRotationSubsystem,m_ArmExtensionSubsystem));
    SmartDashboard.putData("HomeArmExtension", new InstantCommand(m_ArmExtensionSubsystem::setSensorReference, m_ArmExtensionSubsystem));
    SmartDashboard.putData("HomeArmRotation", new InstantCommand(m_ArmRotationSubsystem::setSensorReference, m_ArmRotationSubsystem));
    SmartDashboard.putData("Rotate UP", new InstantCommand(m_ArmRotationSubsystem::rotateUp50, m_ArmRotationSubsystem));
    SmartDashboard.putData("Rotate Down", new InstantCommand(m_ArmRotationSubsystem::rotateDown50, m_ArmRotationSubsystem));
    SmartDashboard.putData("ArmExtend",new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition));

  */
      // Configure default commands
      m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.allianceRelativeDrive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
      m_ArmRotationSubsystem.setDefaultCommand(
        new RunCommand(() -> m_ArmRotationSubsystem.rotate(rightJoystick.getY()), m_ArmRotationSubsystem));
      
  

  SmartDashboard.putNumber("joystickX", 0.0);
  SmartDashboard.putNumber("joystickY", 0.0);
  SmartDashboard.putNumber("encoder version", m_enctest.getDeviceID());
  SmartDashboard.putNumber("encoder",3.14159);
  SmartDashboard.putNumber("InEncode",1234.0);
  Utilities.toSmartDashboard("NearPoseTestPose", new Pose2d(1.84, 8.2, new Rotation2d(Units.degreesToRadians(-90))));
  Utilities.toSmartDashboard("NearPoseTestNear", Utilities.nearPose(new Pose2d(1.84, 8.2, new Rotation2d(Units.degreesToRadians(-90))), 1.0));
}

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /*
     * 2023 Proposed RIGHT joystick button bindings
     * Joystick FWD - drives the robot forward
     * Joystick BACK - drives the robot backwards
     * 1 close the grip (trigger)
     * 2 open the grip (thumb button)
     * 3 Aim at an AprilTag
     * 4 Initiate Balance mode or while pressed
     */
    /*
     * 2023 Proposed LEFT joystick button bindings
     * Joystick FWD - moves set point on arm negative while engaged
     * Joystick Back - moves set point on arm positive while engaged
     * 1 extends the arm while pressed by increasing the set point
     * 2 retracts the arm while pressed by decreasing the set point
     * 3 set ARM ROTATION to LOW and set ARM EXTENSION as appropriate (armpos2)
     * 4 set ARM ROTATION to MID and set ARM EXTENSION as appropriate (armpos3)
     * 5 set ARM ROTATION to GATHER and set ARM EXTENSION as appropriate (armpos1)
     * 6 set ARM ROTATION to HIGH and set ARM EXTENSION as appropriate (armpos4)
     */
    //new JoystickButton(rightJoystick, 1).onTrue(new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem));
    //new JoystickButton(rightJoystick, 2).onTrue(new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem));
    //new JoystickButton(rightJoystick, 3).whileTrue(new FollowApriltagCommand(m_CameraVisionSubsystem, m_DriveTrain)); // Should this lower the arm?
    //new JoystickButton(rightJoystick, 6).onTrue(new BalanceCommandDebugEZ2(m_DriveTrain, m_GyroSubsystem,m_EncoderSubsystem, m_BrakeSubsystem,82.0,0.20));
    
    //new JoystickButton(rightJoystick, 10).onTrue(new ArmEmergencyStop(m_ArmRotationSubsystem, m_ArmExtensionSubsystem));
    //new JoystickButton(rightJoystick, 11).onTrue(new InstantCommand(m_BrakeSubsystem::brake,m_BrakeSubsystem));
    //new JoystickButton(rightJoystick, 12).onTrue(new InstantCommand(m_BrakeSubsystem::unBrake,m_BrakeSubsystem));

    // Left Joystick for Arm Rotation and Extension Control
    //new JoystickButton(leftJoystick, 1).onTrue(new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem));
    //new JoystickButton(leftJoystick, 2).onTrue(new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem));
/*      new JoystickButton(leftJoystick, 3
    ).onTrue( new ConditionalCommand( 
      new SequentialCommandGroup(
        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmLowPosition).withTimeout(3.0),
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition)
      ),
      new SequentialCommandGroup(
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition).withTimeout(3.0)
,        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmLowPosition).withTimeout(3.0)
      ),
      () -> m_ArmRotationSubsystem.getPosition() < ArmConstants.kArmLowPosition
    ));
    new JoystickButton(leftJoystick, 4
    ).onTrue( new ConditionalCommand( 
      new SequentialCommandGroup(
        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmMidPosition).withTimeout(3.0),
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionMidPosition).withTimeout(3.0)
      ),
      new SequentialCommandGroup(
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionMidPosition).withTimeout(3.0)
,        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmMidPosition).withTimeout(3.0)
      ),
      () -> m_ArmRotationSubsystem.getPosition() < ArmConstants.kArmMidPosition
    ));
   */
   //new JoystickButton(leftJoystick, 5).onTrue(new BalanceCommandDebugEZ2(m_DriveTrain, m_GyroSubsystem,m_EncoderSubsystem, m_BrakeSubsystem,82.0,0.55)); 

  /*  new JoystickButton(leftJoystick, 6
    ).onTrue( new ConditionalCommand( 
      new SequentialCommandGroup(
        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmHiPosition).withTimeout(3.0),
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition).withTimeout(3.0)
      ),
      new SequentialCommandGroup(
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition).withTimeout(3.0)
,        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmHiPosition).withTimeout(3.0)
      ),
      () -> m_ArmRotationSubsystem.getPosition() < ArmConstants.kArmHiPosition
    ));
   
   */
   //new JoystickButton(m_driverController,Button.kLeftBumper.value).onTrue(new InstantCommand(() -> m_ArmRotationSubsystem.test_rotate(0.10)));
   //new JoystickButton(m_driverController,Button.kBack.value).whileTrue(new ShooterCommand(m_ShooterSubsystem, 0.3));
    /* 
  
  // Left Joystick for Arm Extension Control Debug
    new JoystickButton(leftJoystick, 7).onTrue(new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition));
    new JoystickButton(leftJoystick, 9).onTrue(new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionMidPosition));
    new JoystickButton(leftJoystick, 11).onTrue(new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition));
    new JoystickButton(leftJoystick, 10).onTrue(new InstantCommand(m_ArmExtensionSubsystem::setSensorReference, m_ArmExtensionSubsystem));
    new JoystickButton(leftJoystick, 12).onTrue(new InstantCommand(m_ArmRotationSubsystem::setSensorReference, m_ArmRotationSubsystem));

    */

/**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  
    /*new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

            
      */
    // This next command is just for testing and should be removed or disabled for game play.
     new JoystickButton(m_driverController, Button.kA.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));
      new JoystickButton(m_driverController, Button.kX.value)
             .whileTrue(new InstantCommand(()->m_ArmRotationSubsystem.disableMotor()));
      new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new InstantCommand(()->m_ArmRotationSubsystem.setPosition(0.0)));
      new JoystickButton(m_driverController, Button.kRightBumper.value)
      //  .whileTrue(new RunCommand(()->m_ArmRotationSubsystem.test_rotate(-0.1)));
      .onTrue(new InstantCommand(()->m_ArmRotationSubsystem.setPosition(3000.0)));
      new JoystickButton(m_driverController, Button.kBack.value)
             .whileTrue(new InstantCommand(()->m_ArmRotationSubsystem.setHome()));
       new JoystickButton(m_driverController, Button.kStart.value)
             .whileTrue(new InstantCommand(()->m_ArmRotationSubsystem.setSensorReference()));


           // .andThen(()->m_ArmRotationSubsystem.test_rotate(0.1));
    // This next command is just for testing and should be removed or disabled for game play. TODO Make sure this is the right way to use the Command
    new JoystickButton(m_driverController, Button.kB.value)
    .whileTrue(new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, 1.46, 1.25, 
    Units.degreesToRadians(240.0)));
    // This command resets the drive train's pose to the current pose from the pose estimator.  It is also for debug
    // although it might be useful during game play to initialize the robot's coordinate system.  That is TBD.
    // This might be better calling m_PoseEstimatorSubsystem.setCurrentPose() instead of resetOdometry
    new JoystickButton(m_driverController, Button.kY.value)
            .onTrue(new InstantCommand(
               // () -> m_robotDrive.setAngleDegrees(m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees()),
               () -> alignDriveTrainToPoseEstimator(),
                m_robotDrive));
                
    SmartDashboard.putData("ResetOdometry",  new RunCommand(() -> m_robotDrive.setAngleDegrees(m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees())));  // For debug without robot
    SmartDashboard.putData("GotoPoseTest",  new RunCommand( () -> new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, 1.46, 1.25, 
    Units.degreesToRadians(240.0))));  // For debug without robot
    SmartDashboard.putData("AlignD2P",  new InstantCommand( () -> alignDriveTrainToPoseEstimator(), m_robotDrive));  // For debug without robot

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  enum AutonomousStrategy {
    LABTEST,
    BLUEALLIANCE,
    REDALLIANCE,
    USEALLIANCE
  }
  public Command getAutonomousCommand() {
    

    AutonomousStrategy autonomousStrategy = AutonomousStrategy.LABTEST;
    autonomousStrategy = AutonomousStrategy.BLUEALLIANCE;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    if (autonomousStrategy == AutonomousStrategy.LABTEST) {
      // Use the current pose estimator's result for the robots actual pose
      //m_robotDrive.resetOdometry(startingPose);
      alignDriveTrainToPoseEstimator();

      // An example trajectory to follow. All units in meters.
      Pose2d startingPose = m_PoseEstimatorSubsystem.getCurrentPose();
      
      //double x=2;
      //double y = 2;
      //double theta = Units.degreesToRadians(-129.0);
      ////x = 1;
      ////y = 0;
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          
          startingPose,
          // Pass through these two interior waypoints, making an 's' curve path
          //List.of(new Translation2d(x+1, y+1), new Translation2d(x+2, y+ -1), new Translation2d(x+3,y+0)),
          List.of(FieldConstants.NearBlueAmp, FieldConstants.NearBandSaw, FieldConstants.NearDriverStation, FieldConstants.NearHammers, FieldConstants.NearNorthDoorToClassroom),
          // End 3 meters straight ahead of where we started, facing forward
          startingPose,
          config);

      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);


      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    } else if ((autonomousStrategy == AutonomousStrategy.USEALLIANCE && Utilities.isBlueAlliance()) || 
               (autonomousStrategy == AutonomousStrategy.BLUEALLIANCE)) {
      // We are in the Blue Alliance.
      //   Drive toward april tag # 6.
      //   Rotate 180 degrees so that we are positioned for shooting.
      //   Back in the last 1 meter to be touching the wall.
      //   Raise the arm to the shooting position.
      //   Run the shooting/outtake motor to score the "note".
      Pose2d targetPose = m_PoseEstimatorSubsystem.getAprilTagPose(AprilTag.BLUE_AMP.id());
      Rotation2d finalRotation = targetPose.getRotation(); // This will put thte back of the robot towards the april tag.
      Rotation2d nearTagRotation = targetPose.getRotation().rotateBy(new Rotation2d(Math.PI)); // This will face the april tag.
      Pose2d finalPose = new Pose2d(targetPose.getX(), targetPose.getY() - 0.5, finalRotation); // Pose for robot to be at the april tag.
      Pose2d nearTagPose = new Pose2d(targetPose.getX(), targetPose.getY() - 1.0, nearTagRotation);
      Pose2d startingPose = m_PoseEstimatorSubsystem.getCurrentPose();
      int lastAprilTagSeen = m_PoseEstimatorSubsystem.lastAprilTagSeen(); // We can use this to be sure we have the right alliance and have decent field coordinates.
      Utilities.toSmartDashboard("AutoTarget", targetPose);
      Utilities.toSmartDashboard("AutoFinal", finalPose);
      Utilities.toSmartDashboard("AutoNearTag", nearTagPose);
      Utilities.toSmartDashboard("AutoStart", startingPose);
      
      // Use the current pose estimator's result for the robots actual pose
      //m_robotDrive.resetOdometry(startingPose);
      alignDriveTrainToPoseEstimator();

      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          
          startingPose,  // We are starting where we are.
          // Pass through these zero interior waypoints, this should probably be something to make sure we dont crash into other robots.
          List.of(),
          nearTagPose,
          config);

      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier  // Should this be the PoseEstimator??
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);
        SequentialCommandGroup fullCommandGroup = new SequentialCommandGroup(
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "SwerveController")),
          swerveControllerCommand.withTimeout(10.0).andThen(() -> m_robotDrive.drive(0, 0, 0, true, true)),
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "GotoPose1")),

          new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, finalPose.getX(), finalPose.getY(), 
            finalPose.getRotation().getRadians()).withTimeout(10.0),
          new ParallelDeadlineGroup(
            CompoundCommands.ScoreNoteInAmp(m_ArmRotationSubsystem, m_ShooterSubsystem),
            new StopRobotMotion(m_robotDrive)
          ),
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "GotoPose2")),
          new GotoPoseCommand(m_PoseEstimatorSubsystem, m_robotDrive, 
            startingPose.getX(), startingPose.getY(), startingPose.getRotation().getRadians()).withTimeout(10.0),
          //new WaitCommand(10.0),
          new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Done"))

 );
        return fullCommandGroup;

    } else if ((autonomousStrategy == AutonomousStrategy.USEALLIANCE && Utilities.isRedAlliance()) ||
               (autonomousStrategy == AutonomousStrategy.REDALLIANCE)) {
      // Perform the Red Alliance autonomous moves.
      // TODO Figure out what the red alliance strategy is exactly...
    } else {
      // Do what we can given we do not know which alliance we are in.
      // Currently it does nothing but stop the drive train which should already be stopped.
      return new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive);
    }
    return new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive); // Should never reach this code.
  }

  // Function to align the PoseEstimator pose and the DriveTrain pose.
  public void alignDriveTrainToPoseEstimator() {
    m_robotDrive.setAngleDegrees(m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees());
    m_robotDrive.resetOdometry(m_PoseEstimatorSubsystem.getCurrentPose());
  }
}