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
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpinConstants;
import frc.robot.Constants.PCMConstants;
import frc.robot.Constants.PositionWheelConstants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.ColorMatcher;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.RampSubsystem;
import frc.robot.subsystems.ShiftSubsystem;
import frc.robot.subsystems.SpinTheWheelSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  public static ColorMatcher m_ColorMatcher = new ColorMatcher();
 //private final CompressorSubsystem m_Compressor = new CompressorSubsystem();
 // private final RampSubsystem m_Ramp = new RampSubsystem();
  private final WinchSubsystem m_WinchSubsystem = new WinchSubsystem();
  public static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static BallSubsystem m_BallSubsystem = new BallSubsystem();
  private final SpinTheWheelSubsystem m_SpinTheWheelSubsystem = new SpinTheWheelSubsystem();
  private final GyroSubsystem m_GyroSubsystem = new GyroSubsystem();
//  private final ShiftSubsystem m_Shifter = new ShiftSubsystem();
  private final HookSubsystem m_HookSubsystem = new HookSubsystem();

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);
  private final Joystick xboxController = new Joystick(OIConstants.kXboxController);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    m_DriveTrain.setDefaultCommand(
      new RunCommand(() -> m_DriveTrain.drive(rightJoystick.getY(), rightJoystick.getX()), m_DriveTrain)
    );
    m_GyroSubsystem.setDefaultCommand(
      new RunCommand(() -> m_GyroSubsystem.periodic(), m_GyroSubsystem)
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

    new JoystickButton(xboxController, Constants.OIConstants.kXboxRBumper).whileHeld(new WinchCommand(m_WinchSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxLBumper).whileHeld(new WinchCommand(m_WinchSubsystem, false));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxStart).whileHeld(new HookCommand(m_HookSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxSelect).whileHeld(new HookCommand(m_HookSubsystem, false));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxYButton).whileHeld(new ElevatorCommand(m_ElevatorSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxXButton).whileHeld(new ElevatorCommand(m_ElevatorSubsystem, false));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxBButton).whileHeld(new BallCommand(m_BallSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxAButton).whileHeld(new BallCommand(m_BallSubsystem, false));
    new JoystickButton(leftJoystick, 2).whenPressed(new SpinCommand(m_SpinTheWheelSubsystem, m_ColorMatcher).withTimeout(SpinConstants.kSpinTimeout));
    new JoystickButton(leftJoystick, 3).whenPressed(new PositionWheelCommand(m_SpinTheWheelSubsystem, m_ColorMatcher).withTimeout(PositionWheelConstants.kPositionWheelTimeout));
//    new JoystickButton(rightJoystick, 3).toggleWhenPressed(new ShiftCommand(m_Shifter));
//    new JoystickButton(xboxController, Constants.OIConstants.kXboxStart).toggleWhenPressed(new RampCommand(m_Ramp));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Autonomous(m_DriveTrain, m_GyroSubsystem);
  }
}
