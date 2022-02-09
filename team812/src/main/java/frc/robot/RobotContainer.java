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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.EncoderSubsystem;
import frc.robot.subsystems.CameraLightSubsystem;
import frc.robot.commands.*;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveTrain m_DriveTrain = new DriveTrain();

  private final WinchSubsystem m_WinchSubsystem = new WinchSubsystem();
  public static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static BallSubsystem m_BallSubsystem = new BallSubsystem();
//  private final ShiftSubsystem m_Shifter = new ShiftSubsystem();
  private final HookSubsystem m_HookSubsystem = new HookSubsystem();

  private final EncoderSubsystem m_EncoderSubsystem = new EncoderSubsystem();
  private final CameraLightSubsystem m_CameraLightSubsystem = new CameraLightSubsystem();

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  private final Joystick xboxController = new Joystick(OIConstants.kXboxController);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if( RobotContainer.m_BlackBox.isSet(OIConstants.kControlBoxSw4) ) {
      System.out.printf("*** Drive mode: drive (squared)\n");
      m_DriveTrain.setDefaultCommand(
        new RunCommand(() -> m_DriveTrain.drive(xboxController.getY(), xboxController.getX()), m_DriveTrain)
      ); 
    } else {
      System.out.printf("*** Drive mode: doge (cubed)\n");
      m_DriveTrain.setDefaultCommand(
        new RunCommand(() -> m_DriveTrain.doge(xboxController.getY(), xboxController.getX()), m_DriveTrain)
      );
    }

   /* m_DriveTrain.setDefaultCommand(
      new RunCommand(() -> m_DriveTrain.midnightDrive(xboxController.getY(), xboxController.getX()), m_DriveTrain));
*/

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(xboxController, Constants.OIConstants.kXboxRBumper).whileHeld(new WinchCommand(m_WinchSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxLBumper).whileHeld(new WinchCommand(m_WinchSubsystem, false));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxStart).whileHeld(new HookCommand(m_HookSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxSelect).whileHeld(new HookCommand(m_HookSubsystem, false));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxYButton).whileHeld(new ElevatorCommand(m_ElevatorSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxXButton).whileHeld(new ElevatorCommand(m_ElevatorSubsystem, false));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxBButton).whileHeld(new BallCommand(m_BallSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxAButton).whileHeld(new BallCommand(m_BallSubsystem, false));
    //new JoystickButton(xboxController, Constants.OIConstants.kXboxAButton).toggleWhenPressed(new StartEndCommand(m_CameraLightSubsystem::on, m_CameraLightSubsystem::off, m_CameraLightSubsystem));
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
