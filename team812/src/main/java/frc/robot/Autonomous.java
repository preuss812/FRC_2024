/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;

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
    ArmExtensionSubsystem m_armExtensionSubsystem = RobotContainer.m_ArmExtensionSubsystem;
    CameraVisionSubsystem m_CameraVisionSubsystem = RobotContainer.m_CameraVisionSubsystem;
    GripperSubsystem m_GripperSubsystem = RobotContainer.m_GripperSubsystem;
    GyroSubsystem m_GyroSubsystem = RobotContainer.m_GyroSubsystem; 
    
    blackBox.readBits();

    addCommands(
      new SequentialCommandGroup(
       // new ArmHomeCommand(m_armSubsystem, m_armExtensionSubsystem),
      //  new TestProfiledPIDCommand(m_CameraVisionSubsystem, m_driveTrain)
      new DriveForwardCommand(m_driveTrain, 0.4, m_GyroSubsystem, null).withTimeout(0.5),
      new DriveByAngleCommand(m_driveTrain, m_GyroSubsystem, 0.5, -90.0), 
      new DriveForwardCommand(m_driveTrain, 0.4, m_GyroSubsystem, null).withTimeout(1.5),
      new DriveByAngleCommand(m_driveTrain, m_GyroSubsystem, 0.5, 90.0) 


       // new FollowApriltagCommand(m_CameraVisionSubsystem, m_driveTrain)
//        new CameraVisionPoseCommand(m_CameraVisionSubsystem, m_driveTrain),
  //      new DriveBackwardCommand(m_driveTrain, 0.5, -0.2).withTimeout(2.0)
      )
    );
  }
}
