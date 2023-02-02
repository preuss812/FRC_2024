/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.ButtonMonitor;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.CameraVisionSubsystem;
import frc.robot.subsystems.CameraLightSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
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
  public static CompressorSubsystem m_Compressor = new CompressorSubsystem();
  public static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  public static BlackBoxSubsystem m_BlackBox = new BlackBoxSubsystem();
  public static BallSubsystem m_BallSubsystem = new BallSubsystem();
  public static ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  public static CameraVisionSubsystem m_CameraVisionSubsystem = new CameraVisionSubsystem();

//  private final ShiftSubsystem m_Shifter = new ShiftSubsystem();

  private final CameraLightSubsystem m_CameraLightSubsystem = new CameraLightSubsystem();

  // Controller definitions
  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystick);

    // Gyro
    private final GyroSubsystem m_GyroSubsystem = new GyroSubsystem();
  
//  private final Joystick xboxController = new Joystick(OIConstants.kXboxController);
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
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
   
    m_ArmSubsystem.unsetHome("RobotContainer");

     m_DriveTrain.setDefaultCommand(
       new RunCommand(() -> m_DriveTrain.preussDrive(rightJoystick.getY(), -rightJoystick.getX()), m_DriveTrain)
     );

    // Default command for the Elevator Subsystem
    m_ElevatorSubsystem.setDefaultCommand(
      new RunCommand( () -> m_ElevatorSubsystem.elevate(-leftJoystick.getY()), m_ElevatorSubsystem)
    );
    //   m_ArmSubsystem.setDefaultCommand(
    //     new RunCommand( ()->m_ArmSubsystem.rotate2(-xboxController.getY()), m_ArmSubsystem)
    //  );

    m_ArmSubsystem.setDefaultCommand(
        new RunCommand( ()->m_ArmSubsystem.rotate2(POV_to_double(rightJoystick.getPOV())), m_ArmSubsystem)
     );

    // Gyro subsystem
    m_GyroSubsystem.setDefaultCommand(
	new RunCommand(() -> m_GyroSubsystem.periodic(), m_GyroSubsystem)
    );


    m_CameraLightSubsystem.off();
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

    // xbox controller
/*    new JoystickButton(xboxController, Constants.OIConstants.kXboxBButton).whileHeld(new BallCommand(m_BallSubsystem, true));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxAButton).whileHeld(new BallCommand(m_BallSubsystem, false));
    new JoystickButton(xboxController, Constants.OIConstants.kXboxXButton).onTrue(
      new SequentialCommandGroup(
        new InstantCommand(m_CameraLightSubsystem::on,m_CameraLightSubsystem),
        new WaitCommand(0.25),
        new CameraVisionCommand(m_CameraVisionSubsystem, m_DriveTrain)
      )
    );
    */

    // Right Joystick for Arm control
    new JoystickButton(rightJoystick, 7).toggleOnTrue(new StartEndCommand(m_CameraLightSubsystem::on, m_CameraLightSubsystem::off, m_CameraLightSubsystem));
    new JoystickButton(rightJoystick, 1).onTrue(new ArmERCommand(m_ArmSubsystem, true));
    new JoystickButton(rightJoystick, 2).onTrue(new ArmERCommand(m_ArmSubsystem, false));

    // This first button takes the arm to the bottom of its travel until
    // the bottom limit switch is trigged. This "Homes" the encoder to a known
    // zero position.
    new JoystickButton(rightJoystick, 11).onTrue(
      new SequentialCommandGroup(
        new ArmERCommand(m_ArmSubsystem, false),
        new WaitCommand(0.25),
        new ArmHomeCommand(m_ArmSubsystem)
      )
    );
    new JoystickButton(rightJoystick, 5).onTrue(new ArmCommand(m_ArmSubsystem, ArmConstants.kArmBallGathering));
    new JoystickButton(rightJoystick, 3).onTrue(
      new SequentialCommandGroup(
        new ArmCommand(m_ArmSubsystem, ArmConstants.kArmPreGrabPosition),
        //new WaitCommand(0.50),
        new ArmERCommand(m_ArmSubsystem, true)
      )
    );
    new JoystickButton(rightJoystick, 4).onTrue(new ArmCommand(m_ArmSubsystem,ArmConstants.kArmHangPosition));
    new JoystickButton(rightJoystick, 6).onTrue(new ArmCommand(m_ArmSubsystem,ArmConstants.kArmTopPositon));
    new JoystickButton(rightJoystick, 8).onTrue(new InstantCommand(m_GyroSubsystem::resetDisplacement, m_GyroSubsystem));
    new JoystickButton(rightJoystick, 9).onTrue(new CameraVisionPoseCommand(m_CameraVisionSubsystem, m_DriveTrain));
    new JoystickButton(rightJoystick, 10).onTrue(new ArmEmergencyStop(m_ArmSubsystem));

    // Left Joystick for Elevator Control
    // Elevator up back
    // Elevator down forward
    new JoystickButton(leftJoystick, 6).onTrue(new ElevatorGripCommand(m_ElevatorSubsystem, true));
    new JoystickButton(leftJoystick, 4).onTrue(new ElevatorGripCommand(m_ElevatorSubsystem, false));
    new JoystickButton(leftJoystick, 7).onTrue(new ArmCommand(m_ArmSubsystem,ArmConstants.kArmHangPosition));
    new JoystickButton(leftJoystick, 8).onTrue(new ArmERCommand(m_ArmSubsystem, true));
    new JoystickButton(leftJoystick, 9).onTrue(new ElevatorGripCommand(m_ElevatorSubsystem, true));
    new JoystickButton(leftJoystick, 10).whileTrue(new BalanceCommand(m_DriveTrain, m_GyroSubsystem));
    new JoystickButton(leftJoystick, 11).whileTrue(new DriveForwardCommand(m_DriveTrain, 0.25, m_GyroSubsystem, null));
    new JoystickButton(leftJoystick, 12).onTrue(new InstantCommand(m_ElevatorSubsystem::enable_elevator,m_ElevatorSubsystem));

    // Toggle Home boolean for the arm - this should not be used in competition
    //new JoystickButton(leftJoystick, 7).toggleonTrue(new StartEndCommand(m_ArmSubsystem::setHome,m_ArmSubsystem::unsetHome2,m_ArmSubsystem));

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
