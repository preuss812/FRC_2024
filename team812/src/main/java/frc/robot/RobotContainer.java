/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.EncoderSubsystem;
import frc.robot.subsystems.DigitalIOSubsystem;
import frc.robot.subsystems.BrakeSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.CameraVisionSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.commands.*;

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
  private final DriveTrain m_DriveTrain = new DriveTrain();
  public static CompressorSubsystem m_Compressor = new CompressorSubsystem();
  public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static ArmRotationSubsystem m_ArmRotationSubsystem = new ArmRotationSubsystem(); // This is arm rotation - dph
  public static ArmExtensionSubsystem m_ArmExtensionSubsystem = new ArmExtensionSubsystem();
  public static CameraVisionSubsystem m_CameraVisionSubsystem = new CameraVisionSubsystem();
  public static BrakeSubsystem m_BrakeSubsystem = new BrakeSubsystem();
  public static GripperSubsystem m_GripperSubsystem = new GripperSubsystem();
  public static EncoderSubsystem m_EncoderSubsystem = new EncoderSubsystem();
  //public static DigitalIOSubsystem m_DigitalIOSubsystem = new DigitalIOSubsystem();

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);

  // Gyro
  public static GyroSubsystem m_GyroSubsystem = new GyroSubsystem();

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

    m_ArmRotationSubsystem.unsetHome("RobotContainer");

    m_DriveTrain.setDefaultCommand(
        new RunCommand(() -> m_DriveTrain.preussDrive(rightJoystick.getY(), -rightJoystick.getX()), m_DriveTrain));

    m_ArmRotationSubsystem.setDefaultCommand(
        new RunCommand(() -> m_ArmRotationSubsystem.rotate(leftJoystick.getY()), m_ArmRotationSubsystem));

    m_ArmExtensionSubsystem.setDefaultCommand(
        new RunCommand(() -> m_ArmExtensionSubsystem.testMoveInOut(POV_to_double(leftJoystick.getPOV())),
            m_ArmExtensionSubsystem));

    // Gyro subsystem
    m_GyroSubsystem.setDefaultCommand(
        new RunCommand(() -> m_GyroSubsystem.periodic(), m_GyroSubsystem));

    // Configure the button bindings
    configureButtonBindings();

    // for debug: put subsystem info on the shuffleboard. - remove before competition.
    SmartDashboard.putData(m_ArmExtensionSubsystem);
    SmartDashboard.putData(m_ArmRotationSubsystem);
    SmartDashboard.putData("ArmRotate", new ArmCommand(m_ArmRotationSubsystem, 1000));
    SmartDashboard.putData("ArmHomeCommand", new ArmHomeCommand(m_ArmRotationSubsystem,m_ArmExtensionSubsystem));
    SmartDashboard.putData("HomeArmExtension", new InstantCommand(m_ArmExtensionSubsystem::setSensorReference, m_ArmExtensionSubsystem));
    SmartDashboard.putData("HomeArmRotation", new InstantCommand(m_ArmRotationSubsystem::setSensorReference, m_ArmRotationSubsystem));
    SmartDashboard.putData("Rotate UP", new InstantCommand(m_ArmRotationSubsystem::rotateUp50, m_ArmRotationSubsystem));
    SmartDashboard.putData("Rotate Down", new InstantCommand(m_ArmRotationSubsystem::rotateDown50, m_ArmRotationSubsystem));
    SmartDashboard.putData("ArmExtend",new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition));
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
    new JoystickButton(rightJoystick, 1).onTrue(new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem));
    new JoystickButton(rightJoystick, 2).onTrue(new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem));
    new JoystickButton(rightJoystick, 3).whileTrue(new FollowApriltagCommand(m_CameraVisionSubsystem, m_DriveTrain)); // Should this lower the arm?
    new JoystickButton(rightJoystick, 4).whileTrue(new BalanceCommand(m_DriveTrain, m_GyroSubsystem));                // Should this lower the arm?
    new JoystickButton(rightJoystick, 5).whileTrue(new BalanceCommandDebug(m_DriveTrain, m_GyroSubsystem,m_EncoderSubsystem)); 
    
    new JoystickButton(rightJoystick, 8)
        .onTrue(new InstantCommand(m_GyroSubsystem::resetDisplacement, m_GyroSubsystem));
    new JoystickButton(rightJoystick, 10).onTrue(new ArmEmergencyStop(m_ArmRotationSubsystem, m_ArmExtensionSubsystem));
    new JoystickButton(rightJoystick, 11).onTrue(new InstantCommand(m_BrakeSubsystem::brake,m_BrakeSubsystem));
    new JoystickButton(rightJoystick, 12).onTrue(new InstantCommand(m_BrakeSubsystem::unBrake,m_BrakeSubsystem));

    // Left Joystick for Arm Rotation and Extension Control
    new JoystickButton(leftJoystick, 1).onTrue(new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem));
    new JoystickButton(leftJoystick, 2).onTrue(new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem));
     new JoystickButton(leftJoystick, 3
    ).onTrue( new ConditionalCommand( 
      new SequentialCommandGroup(
        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmLowPosition).withTimeout(3.0),
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition)
      ),
      new SequentialCommandGroup(
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition)
,        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmLowPosition).withTimeout(3.0)
      ),
      () -> m_ArmRotationSubsystem.getPosition() < ArmConstants.kArmLowPosition
    ));
    new JoystickButton(leftJoystick, 4
    ).onTrue( new ConditionalCommand( 
      new SequentialCommandGroup(
        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmMidPosition).withTimeout(3.0),
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionMidPosition)
      ),
      new SequentialCommandGroup(
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionMidPosition)
,        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmMidPosition).withTimeout(3.0)
      ),
      () -> m_ArmRotationSubsystem.getPosition() < ArmConstants.kArmMidPosition
    ));
    new JoystickButton(leftJoystick, 5).whileTrue(new BalanceCommandDebugEZ(m_DriveTrain, m_GyroSubsystem,m_EncoderSubsystem, m_BrakeSubsystem)); 

    new JoystickButton(leftJoystick, 6
    ).onTrue( new ConditionalCommand( 
      new SequentialCommandGroup(
        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmHiPosition).withTimeout(3.0),
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition).withTimeout(5)
      ),
      new SequentialCommandGroup(
        new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition)
,        new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmHiPosition).withTimeout(3.0)
      ),
      () -> m_ArmRotationSubsystem.getPosition() < ArmConstants.kArmHiPosition
    ));
  
  // Left Joystick for Arm Extension Control Debug
    new JoystickButton(leftJoystick, 7).onTrue(new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition));
    new JoystickButton(leftJoystick, 9).onTrue(new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionMidPosition));
    new JoystickButton(leftJoystick, 11).onTrue(new ArmExtensionCommand(m_ArmExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition));
    new JoystickButton(leftJoystick, 10).onTrue(new InstantCommand(m_ArmExtensionSubsystem::setSensorReference, m_ArmExtensionSubsystem));
    new JoystickButton(leftJoystick, 12).onTrue(new InstantCommand(m_ArmRotationSubsystem::setSensorReference, m_ArmRotationSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Autonomous(m_DriveTrain);
  }
}
