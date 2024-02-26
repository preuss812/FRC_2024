/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.ArmConstants;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants.AprilTag;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.NoteIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.CameraVisionSubsystem;
import frc.robot.commands.ArmHomeCommand;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.commands.GotoAmpCommand;
import frc.robot.commands.GotoPoseCommand;
import frc.robot.commands.NoteIntakeCommand;
import frc.robot.commands.ScoreNoteInAmp;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.StopRobotMotion;
import frc.robot.commands.TakeInNoteCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.GotoSourceCommand;




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
  public final static DriveSubsystemSRX m_robotDrive = new DriveSubsystemSRX();

 // public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static CameraVisionSubsystem m_CameraVisionSubsystem = new CameraVisionSubsystem();
  //public static EncoderSubsystem m_EncoderSubsystem = new EncoderSubsystem();
  public static PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem( m_CameraVisionSubsystem.camera, m_robotDrive);
  public static ArmRotationSubsystem m_ArmRotationSubsystem = new ArmRotationSubsystem();
  public static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public static NoteIntakeSubsystem m_NoteIntakeSubsystem = new NoteIntakeSubsystem();
  public static WinchSubsystem m_WinchSubsystem = new WinchSubsystem();

  //public static DigitalIOSubsystem m_DigitalIOSubsystem = new DigitalIOSubsystem();

  // Controller definitions
  //private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  
  //public static CANcoder m_enctest = new CANcoder(38);

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

    // SmartDashboard.putData("HomeArmRotation", new ArmHomeCommand(m_ArmRotationSubsystem));

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
        m_robotDrive)
    );

    m_ArmRotationSubsystem.setDefaultCommand(
      new RunCommand(() -> m_ArmRotationSubsystem.rotate(-rightJoystick.getY()), m_ArmRotationSubsystem)
    );
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


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  
   
    // This next command is just for testing and should be removed or disabled for game play.
    //new JoystickButton(m_driverController, Button.kBack.value)
    //  .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
      .onTrue(new ScoreNoteInAmp(m_ArmRotationSubsystem, m_ShooterSubsystem));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .whileTrue(new TakeInNoteCommand(m_NoteIntakeSubsystem, m_ShooterSubsystem));

    new JoystickButton(m_driverController, Button.kB.value)
      .whileTrue(new GotoSourceCommand(m_robotDrive, m_PoseEstimatorSubsystem));

    new JoystickButton(m_driverController, Button.kX.value)
      .whileTrue(new GotoAmpCommand(m_robotDrive, m_PoseEstimatorSubsystem));

      //new JoystickButton(m_driverController, Button.kRightBumper.value)
    //  .onTrue(new InstantCommand(()->m_ArmRotationSubsystem.setPosition(3000.0)));

    new JoystickButton(m_driverController, Button.kStart.value)
      .onTrue(new ArmHomeCommand(m_ArmRotationSubsystem));

    // This command resets the drive train's pose to the current pose from the pose estimator.  It is also for debug
    // although it might be useful during game play to initialize the robot's coordinate system.  That is TBD.
    // This might be better calling m_PoseEstimatorSubsystem.setCurrentPose() instead of resetOdometry
    new JoystickButton(m_driverController, Button.kBack.value)
            .onTrue(new InstantCommand(
               // () -> m_robotDrive.setAngleDegrees(m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees()),
               () -> alignDriveTrainToPoseEstimator(),
                m_robotDrive));
                
    //SmartDashboard.putData("AlignD2P",  new InstantCommand( () -> alignDriveTrainToPoseEstimator(), m_robotDrive));  // For debug
    /* Debugging below */
    List<Translation2d> blueAmpPlan = Utilities.planBlueAmpTrajectory(new Pose2d(0,0,new Rotation2d(0)));
    SmartDashboard.putString("blueampplan", blueAmpPlan.toString());
    List<Translation2d> blueAmpPlan2 = Utilities.planBlueAmpTrajectory(new Pose2d(16,0,new Rotation2d(0)));
    SmartDashboard.putString("blueampplan2", blueAmpPlan2.toString());
    for (int i = 1; i <= 16; i+= 2) {
      for (int j = 1; j <= 8; j+= 2) {
        blueAmpPlan = Utilities.planBlueAmpTrajectory(new Pose2d(16,0,new Rotation2d(0)));
      }
    }
    // Test of POV button rotate to 180 (ie toward the alliance Speaker).
    POVButton x = new POVButton(m_driverController, 180);
    x.onTrue(
        new GotoPoseCommand(
          m_PoseEstimatorSubsystem,
          m_robotDrive, 
          Utilities.Pose180(m_PoseEstimatorSubsystem.getCurrentPose()) 
        )
    ).debounce(0.2);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Autonomous(this);
  }
   
  // Function to align the PoseEstimator pose and the DriveTrain pose.
  public void alignDriveTrainToPoseEstimator() {
    m_robotDrive.setAngleDegrees(m_PoseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees());
    m_robotDrive.resetOdometry(m_PoseEstimatorSubsystem.getCurrentPose());
  }
}