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
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.ColorMatcher;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.RampSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  private final ColorMatcher m_ColorMatcher = new ColorMatcher();
  private final CompressorSubsystem m_Compressor = new CompressorSubsystem();
  private final RampSubsystem m_Ramp = new RampSubsystem();
  private final WinchSubsystem m_Winch = new WinchSubsystem();
  private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem();
  public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  private final BallSubsystem m_Ball = new BallSubsystem();

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  private final Joystick xboxController = new Joystick(OIConstants.kXboxController);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    m_DriveTrain.setDefaultCommand(
      new RunCommand(() -> m_DriveTrain.drive(rightJoystick.getX(), rightJoystick.getY()), m_DriveTrain)
    );
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

    new JoystickButton(rightJoystick, 1).whileHeld(new WinchCommand(m_Winch, true));
    new JoystickButton(rightJoystick, 2).whileHeld(new WinchCommand(m_Winch, false));
    new JoystickButton(leftJoystick, 1).whileHeld(new ElevatorCommand(m_Elevator, true));
    new JoystickButton(leftJoystick, 2).whileHeld(new ElevatorCommand(m_Elevator, false));
    new JoystickButton(leftJoystick, 10).whileHeld(new BallCommand(m_Ball, true));
    new JoystickButton(leftJoystick, 11).whileHeld(new BallCommand(m_Ball, false));

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
