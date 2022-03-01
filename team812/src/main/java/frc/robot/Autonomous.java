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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  public DriveTrain m_driveTrain;
  // public GyroSubsystem m_gyro;

  // public Autonomous(DriveTrain subsystem, GyroSubsystem gyro) {
  public Autonomous(DriveTrain subsystem) {

    m_driveTrain = subsystem;

    System.out.printf("*** Entering Autonomous mode\n");

    BlackBoxSubsystem blackBox = RobotContainer.m_BlackBox;
    ArmSubsystem m_armSubsystem = RobotContainer.m_ArmSubsystem;
    BallSubsystem m_ballSubsystem = RobotContainer.m_BallSubsystem;
    
    if(blackBox.isSwitchCenter()) {      // middle position on the field
      System.out.printf("*** Autonomous switch is CENTER\n");

      addCommands(
        new SequentialCommandGroup(
          new ArmHomeCommand(m_armSubsystem),
          new ArmCommand(m_armSubsystem,ArmConstants.kArmScorePosition).withTimeout(0.5),
          new ArmERCommand(m_armSubsystem, true).withTimeout(0.5),
          new BallCommand(m_ballSubsystem, true).withTimeout(0.5),
          new DriveForwardCommand(m_driveTrain, -0.5).withTimeout(0.10)
        )
      );
    } else if (blackBox.isSwitchLeft()){
  //  else if( x >= 0.5 ) {
      // left position on the field
      System.out.printf("*** Autonomous switch is LEFT\n");
      /*addCommands(
        new ParallelCommandGroup(
          new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5)));
          new SequentialCommandGroup(   
            new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5),
            new DriveRightInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0),
            new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0),
            new DriveLeftInPlaceCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0),
            new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5)
          )
        )
      ); */

    } else if( blackBox.isSwitchRight() ) {
      // right position on the field
      System.out.printf("*** Autonomous switch is RIGHT\n");
/*        addCommands(
          new ParallelCommandGroup(
            new ElevatorCommand(RobotContainer.m_ElevatorSubsystem, true).withTimeout(4.5)));
            new SequentialCommandGroup(
              new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0),
              new DriveByAngleCommand(m_subsystem, m_gyro, 0.6, -45.0).withTimeout(1.0),
              new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.80),
              new DriveByAngleCommand(m_subsystem, m_gyro, 0.6, 45.0).withTimeout(1.0),
              new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(2.0)
            )
          )
        ); */
      } else {
        System.out.printf("*** Autonomous driveforward null\n");
      //addCommands(new DriveForwardCommand(m_subsystem, m_gyro, 0.6).withTimeout(1.5));
    }
  }
}
