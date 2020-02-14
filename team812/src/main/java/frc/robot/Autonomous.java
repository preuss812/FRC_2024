/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  public DriveTrain m_subsystem;
  public GyroSubsystem m_gyro;

  public Autonomous(DriveTrain subsystem, GyroSubsystem gyro) {
    m_subsystem = subsystem;
    m_gyro = gyro;
/*    addCommands (new FunctionalCommand(
      ()-> m_subsystem.drive(0.0, 0.0), 
      ()-> m_subsystem.drive(0.5, 0.0),
      ()-> m_subsystem.drive(0.0, 0.0),
      ,
      m_subsystem
    ).withTimeout(1.0));
    */
    addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(5.5));
    addCommands(new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(3.0));
    addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(3.0));
    addCommands(new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(3.0));
    // addCommands( );
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // super();
  }
}
