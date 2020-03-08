/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.Constants.OIConstants;

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


    m_gyro.reset();

    System.out.printf("*** Entering Autonomous mode\n");


    NetworkTableEntry xEntry;
    NetworkTableEntry yEntry;

    double x;
    double y;

    BlackBoxSubsystem blackBox;
    blackBox = RobotContainer.m_BlackBox;


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
    
    if(blackBox.isSwitchCenter()) {      // middle position on the field
      addCommands(
        new ParallelCommandGroup(
          new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.3),
          new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5)
        )
      );
      if(blackBox.isSet(OIConstants.kControlBoxSw3)) {
        addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
        addCommands(new DriveForwardCommand(m_subsystem, m_gyro, -0.5).withTimeout(2.0));
      }
    } else if (blackBox.isSwitchLeft()){
  //  else if( x >= 0.5 ) {
      // left position on the field

      addCommands(
        new ParallelCommandGroup(
          new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5),
          new SequentialCommandGroup(   
            new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0),
            new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5),
            new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.80),
            new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5),
            new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(4)
          )
        )
      );
      if(blackBox.isSet(OIConstants.kControlBoxSw3)) {
        addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
        addCommands(new DriveForwardCommand(m_subsystem, m_gyro, -0.5).withTimeout(2.0));
      }
    } else if( blackBox.isSwitchRight() ) {
      // right position on the field
        addCommands(
          new ParallelCommandGroup(
            new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5),
            new SequentialCommandGroup(
              new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0),
              new DriveByAngleCommand(m_subsystem, m_gyro, 0.6, -45.0).withTimeout(1.0),
              new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.80),
              new DriveByAngleCommand(m_subsystem, m_gyro, 0.6, 45.0).withTimeout(1.0),
              new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0)
            )
          )
        );
        if(blackBox.isSet(OIConstants.kControlBoxSw3)) {
          addCommands(new BallCommand(RobotContainer.m_BallSubsystem, false).withTimeout(2.0));
          addCommands(new DriveForwardCommand(m_subsystem, m_gyro, -0.6).withTimeout(2.0));
        }
      } else {
      addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5));
    }
  }
}
