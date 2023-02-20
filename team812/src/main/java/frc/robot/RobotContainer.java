/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.DriveTrain;
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
  public static GripperSubsystem m_GripperSubsystem = new GripperSubsystem();

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
    SmartDashboard.putNumber("POV", pov);
    SmartDashboard.putNumber("POV_Out", result);
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
        new RunCommand(() -> m_ArmRotationSubsystem.test_rotate(POV_to_double(rightJoystick.getPOV())),
            m_ArmRotationSubsystem));
            
    m_ArmExtensionSubsystem.setDefaultCommand(
        new RunCommand(() -> m_ArmExtensionSubsystem.test_move_in_out(POV_to_double(leftJoystick.getPOV())),
            m_ArmExtensionSubsystem));

    // Gyro subsystem
    m_GyroSubsystem.setDefaultCommand(
        new RunCommand(() -> m_GyroSubsystem.periodic(), m_GyroSubsystem));

    // Configure the button bindings
    configureButtonBindings();
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
    new JoystickButton(rightJoystick, 3).whileTrue(new FollowApriltagCommand(m_CameraVisionSubsystem, m_DriveTrain));
    // This first button takes the arm to the bottom of its travel until
    // the bottom limit switch is trigged. This "Homes" the encoder to a known
    // zero position.
    new JoystickButton(rightJoystick, 11).onTrue(
        new SequentialCommandGroup(
            new ArmERCommand(m_ArmRotationSubsystem, false),
            new WaitCommand(0.25),
            new ArmHomeCommand(m_ArmRotationSubsystem, m_ArmExtensionSubsystem)));
    new JoystickButton(rightJoystick, 5).onTrue(new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmBallGathering));
    new JoystickButton(rightJoystick, 3).onTrue(
        new SequentialCommandGroup(
            new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmPreGrabPosition),
            // new WaitCommand(0.50),
            new ArmERCommand(m_ArmRotationSubsystem, true)));
    
  
    new JoystickButton(rightJoystick, 8)
        .onTrue(new InstantCommand(m_GyroSubsystem::resetDisplacement, m_GyroSubsystem));
    // new JoystickButton(rightJoystick, 9).onTrue(new
    // CameraVisionPoseCommand(m_CameraVisionSubsystem, m_DriveTrain));
    new JoystickButton(rightJoystick, 10).onTrue(new ArmEmergencyStop(m_ArmRotationSubsystem));

    // Left Joystick for Arm Rotation and Extension Control
    new JoystickButton(leftJoystick, 3).onTrue(new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmLowPosition));
    new JoystickButton(leftJoystick, 4).onTrue(new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmMidPosition));
    new JoystickButton(leftJoystick, 6).onTrue(new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmHiPosition));new JoystickButton(leftJoystick, 6).onTrue(new TurnRightBB2(m_DriveTrain)); // No Elevator for 2023.
    // new JoystickButton(leftJoystick, 4).onTrue(new
    // ElevatorGripCommand(m_ElevatorSubsystem, false)); // No Elevator for 2023.
    new JoystickButton(leftJoystick, 7).onTrue(new ArmCommand(m_ArmRotationSubsystem, ArmConstants.kArmHangPosition));
    new JoystickButton(leftJoystick, 8).onTrue(new ArmERCommand(m_ArmRotationSubsystem, true));
    // new JoystickButton(leftJoystick, 9).onTrue(new
    // ElevatorGripCommand(m_ElevatorSubsystem, true)); // No Elevator for 2023.
    new JoystickButton(leftJoystick, 9).onTrue(new FollowApriltagCommandBB(m_CameraVisionSubsystem, m_DriveTrain)); // Optimize
    new JoystickButton(leftJoystick, 12).onTrue(new InstantCommand(m_ArmRotationSubsystem::setSensorReference, m_ArmRotationSubsystem));
                                                                                                                        // controller
                                                                                                                    // using
                                                                                                                    // Black
                                                                                                                    // Box
    new JoystickButton(leftJoystick, 10).whileTrue(new BalanceCommand(m_DriveTrain, m_GyroSubsystem));
    new JoystickButton(leftJoystick, 11).whileTrue(new DriveForwardCommand(m_DriveTrain, 0.25, m_GyroSubsystem, null));
    // new JoystickButton(leftJoystick, 12).onTrue(new
    // InstantCommand(m_ElevatorSubsystem::enable_elevator,m_ElevatorSubsystem)); //
    // No Elevator for 2023.

    // Toggle Home boolean for the arm - this should not be used in competition
    // new JoystickButton(leftJoystick, 7).toggleonTrue(new
    // StartEndCommand(m_ArmSubsystem::setHome,m_ArmSubsystem::unsetHome2,m_ArmSubsystem));

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
