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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.networktables.*;

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

    System.out.printf("*** Entering Autonomous mode\n");

    NetworkTableEntry xEntry;
    NetworkTableEntry yEntry;

    double x;
    double y;
     /* NetworkTable.setClientMode();
      NetworkTable.setTeam(812);
      NetworkTable.setIPAdress("roborio-812-frc.local");
      NetworkTable.initialize();
      */
     

    NetworkTableInstance inst=NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("OpenSight");
    xEntry = table.getEntry("x");
    yEntry = table.getEntry("y");
    x = xEntry.getDouble(2.0);
    y = yEntry.getDouble(2.0);
    System.out.printf("camera (x,y) = (%f, %f)\n", x,y);
    
/*    addCommands (new FunctionalCommand(
      ()-> m_subsystem.drive(0.0, 0.0), 
      ()-> m_subsystem.drive(0.5, 0.0),
      ()-> m_subsystem.drive(0.0, 0.0),
      ,
      m_subsystem
    ).withTimeout(1.0));
    */
    if( x >= -0.4 && x < 0.4) {
      // middle position on the field
      addCommands(new ParallelCommandGroup(
        new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(3.2),
        new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5)
      ));
      /*
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(3.2));
      addCommands(new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5));
      */
      addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
    } else if( x >= 0.5 ) {
      // left position on the field
      /*
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.0));
      addCommands(new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5));
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.60));
      addCommands(new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5));
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(3.0));
      addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, -0.5).withTimeout(2.0));
      */
      addCommands(new ParallelCommandGroup(
      new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5),
      new SequentialCommandGroup(
          
      new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.0),
      new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5),
      new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.80),
      new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5),
      new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(4)
        )
      )
      );
      addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, -0.5).withTimeout(2.0));
    } else if( x <= -0.5 ) {
      // right position on the field
      /*
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.0));
      addCommands(new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5));
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.60));
      addCommands(new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5));
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(3.0));
      addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, -0.5).withTimeout(2.0));
      */
      addCommands(new ParallelCommandGroup(
        new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5),
        new SequentialCommandGroup(
          
        new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.0),
        new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5),
        new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(2.80),
        new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.7),
        new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(4.0)
        )
        )
        );
        addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
        addCommands(new DriveForwardCommand(m_subsystem, m_gyro, -0.5).withTimeout(2.0));
    } else {
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(1.0));
    }
      
    /*
    addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(5.5));
    addCommands(new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(3.0));
    addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.5).withTimeout(3.0));
    addCommands(new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(3.0));
    */

    // addCommands( );
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // super();
  }
}
