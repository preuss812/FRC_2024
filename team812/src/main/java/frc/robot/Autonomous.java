/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.aruco.GridBoard;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;

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
    ArmRotationSubsystem m_armSubsystem = RobotContainer.m_ArmRotationSubsystem;
    ArmExtensionSubsystem m_armExtensionSubsystem = RobotContainer.m_ArmExtensionSubsystem;
    CameraVisionSubsystem m_CameraVisionSubsystem = RobotContainer.m_CameraVisionSubsystem;
    GripperSubsystem m_GripperSubsystem = RobotContainer.m_GripperSubsystem;
    GyroSubsystem m_GyroSubsystem = RobotContainer.m_GyroSubsystem;

    blackBox.readBits();

    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem),                             // The grip is already closed. Tell it so stay closed.
            new ArmHomeCommand(m_armSubsystem, m_armExtensionSubsystem),                                      // Set the coordinates for the arm rotation and extension to 0 for both.
            new ArmCommand(m_armSubsystem, ArmConstants.kArmLowPosition),                                     // Rotate the arm out to allow motion
            new ArmExtensionCommand(m_armExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition), // Extend the arm to the low, gathering position.
            // new TestProfiledPIDCommand(m_CameraVisionSubsystem, m_driveTrain)
            new DriveForwardCommand(m_driveTrain, 0.2, m_GyroSubsystem, null).withTimeout(0.5),
            //new DriveByAngleCommand(m_driveTrain, m_GyroSubsystem, 0.2, -90.0),
            //new DriveForwardCommand(m_driveTrain, 0.2, m_GyroSubsystem, null).withTimeout(1.5),
            //new DriveByAngleCommand(m_driveTrain, m_GyroSubsystem, 0.2, 90.0),
            new ArmCommand(m_armSubsystem, ArmConstants.kArmHiPosition),                                     // Rotate the arm up to score the maximum points.
            new ArmExtensionCommand(m_armExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition), // Extend the arm to the proper scoring height.
            new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem)                              // Release the code/cube to score.                         

        // new FollowApriltagCommand(m_CameraVisionSubsystem, m_driveTrain)
        // new CameraVisionPoseCommand(m_CameraVisionSubsystem, m_driveTrain),
        // new DriveBackwardCommand(m_driveTrain, 0.5, -0.2).withTimeout(2.0)
        ));
  }
}
