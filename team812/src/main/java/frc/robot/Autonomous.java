/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import org.opencv.aruco.GridBoard;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    boolean balancing = false;
    boolean readingBlackBoxSwitch = false;

    m_driveTrain = subsystem;

    System.out.printf("*** Entering Autonomous mode\n");

    BlackBoxSubsystem blackBox = RobotContainer.m_BlackBox;
    ArmRotationSubsystem m_armSubsystem = RobotContainer.m_ArmRotationSubsystem;
    ArmExtensionSubsystem m_armExtensionSubsystem = RobotContainer.m_ArmExtensionSubsystem;
    // CameraVisionSubsystem m_CameraVisionSubsystem = RobotContainer.m_CameraVisionSubsystem;
    BrakeSubsystem m_BrakeSubsystem = RobotContainer.m_BrakeSubsystem;
    GripperSubsystem m_GripperSubsystem = RobotContainer.m_GripperSubsystem;
    GyroSubsystem m_GyroSubsystem = RobotContainer.m_GyroSubsystem;
    EncoderSubsystem m_encoderSubsystem = RobotContainer.m_EncoderSubsystem;

    blackBox.readBits();
    if (readingBlackBoxSwitch) {
      if (RobotContainer.m_BlackBox.isSwitchLeft()) {
        balancing = true;
      } else if (RobotContainer.m_BlackBox.isSwitchCenter()) {
        balancing = false;
      } else {
        balancing = false; 
      }
    }
/* 
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem),                             // The grip is already closed. Tell it so stay closed.
            new ArmHomeCommand(m_armSubsystem, m_armExtensionSubsystem),                                      // Set the coordinates for the arm rotation and extension to 0 for both.
            new ArmCommand(m_armSubsystem, ArmConstants.kArmLowPosition), // untuck                                     // Rotate the arm out to allow motion
            //new ArmExtensionCommand(m_armExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition), // Extend the arm to the low, gathering position.
            // new TestProfiledPIDCommand(m_CameraVisionSubsystem, m_driveTrain)
            new ParallelCommandGroup(
              new ArmCommand(m_armSubsystem, ArmConstants.kArmHiPosition),     
              new DriveDistanceCommand(m_driveTrain, 0.2, 24.0, m_GyroSubsystem, m_encoderSubsystem),
              new ArmExtensionCommand(m_armExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition) // Extend the arm to the proper scoring height.
            ).withTimeout(5),
            //new DriveByAngleCommand(m_driveTrain, m_GyroSubsystem, 0.2, -90.0),
            //new DriveForwardCommand(m_driveTrain, 0.2, m_GyroSubsystem, null).withTimeout(1.5),
            //new DriveByAngleCommand(m_driveTrain, m_GyroSubsystem, 0.2, 90.0),
            //new ArmCommand(m_armSubsystem, ArmConstants.kArmHiPosition),                                   // Rotate the arm up to score the maximum points.
            new WaitCommand(1),
            new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem),                             // Release the code/cube to score.
            new ParallelCommandGroup(
              new DriveDistanceCommand(m_driveTrain, 0.2, -24.0, m_GyroSubsystem, m_encoderSubsystem),
              new ArmExtensionCommand(m_armExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition), // Return retract arm again.
              //new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem),    
              new ArmCommand(m_armSubsystem, ArmConstants.kArmLowPosition)
            ).withTimeout(5),
            //new DriveDistanceCommand(m_driveTrain, 0.2, -(24.0), m_GyroSubsystem, m_encoderSubsystem),
            new DriveDistanceCommand(m_driveTrain, 0.25, -(2.5 * 12.0), m_GyroSubsystem, m_encoderSubsystem),
            new BalanceCommandDebug(m_driveTrain, m_GyroSubsystem)
        // new FollowApriltagCommand(m_CameraVisionSubsystem, m_driveTrain)
        // new CameraVisionPoseCommand(m_CameraVisionSubsystem, m_driveTrain),
        // new DriveBackwardCommand(m_driveTrain, 0.5, -0.2).withTimeout(2.0)
        ));
        */
        if (balancing) {
          // Commands to balance by driving 82 inches and slamming on the brakes.
          addCommands(
          new SequentialCommandGroup(
              new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem),                             // The grip is already closed. Tell it so stay closed.
              new InstantCommand(m_BrakeSubsystem::unBrake,m_BrakeSubsystem),  
              new ArmHomeCommand(m_armSubsystem, m_armExtensionSubsystem),                                      // Set the coordinates for the arm rotation and extension to 0 for both.
              new ArmCommand(m_armSubsystem, ArmConstants.kArmAutonomous), // untuck                           // Rotate the arm out to allow motion
              //new DriveDistanceCommand(m_driveTrain, 0.45, (36.0), m_GyroSubsystem, m_encoderSubsystem), // 0.25 was not enough
              //new BalanceCommandDebug(m_driveTrain, m_GyroSubsystem, m_encoderSubsystem),
              new BalanceCommandDebugEZ2(m_driveTrain, m_GyroSubsystem, m_encoderSubsystem, m_BrakeSubsystem, 80, 0.4) // Drive straight 82 inches at 55% speed and stop
              ));
        } else {
          // Commands to score a purple cube and prepare to score more game pieces
          addCommands(
          new SequentialCommandGroup(
              new InstantCommand(m_GripperSubsystem::closeGrip,m_GripperSubsystem),                               // The grip is already closed. Tell it so stay closed.
              new InstantCommand(m_BrakeSubsystem::unBrake,m_BrakeSubsystem),                                     // Make sure the brake is not engaged.
              new ArmHomeCommand(m_armSubsystem, m_armExtensionSubsystem),                                        // Set the coordinates for the arm rotation and extension to 0 for both.
              // new ArmCommand(m_armSubsystem, ArmConstants.kArmAutonomous), // untuck                              // Rotate the arm out to allow motion
              new ArmCommand(m_armSubsystem, ArmConstants.kArmHighCubePosition).withTimeout(5),                                  // Raise the arm up high.
              new ParallelCommandGroup(
                new DriveDistanceCommand(m_driveTrain, 0.30, (12.0), m_GyroSubsystem, m_encoderSubsystem),   // drive forware 12 inches to score.
                new ArmExtensionCommand(m_armExtensionSubsystem, ArmExtensionConstants.kArmExtensionHiPosition)    // Extend arm to score
              ),             
              new ArmCommand(m_armSubsystem, ArmConstants.kArmHighCubeReleasePosition).withTimeout(2),   // Rotate down slightly to be inside the box
              new InstantCommand(m_GripperSubsystem::openGrip,m_GripperSubsystem),                                // Release the cube.
              new WaitCommand(0.25),                                                                     // Allow time for the grippers to release
              new DriveDistanceCommand(m_driveTrain, 0.30, (-12.0), m_GyroSubsystem, m_encoderSubsystem),  // Back up 12 inches.
              new ParallelCommandGroup(
                new ArmExtensionCommand(m_armExtensionSubsystem, ArmExtensionConstants.kArmExtensionLowPosition),  // Retract and lower the arm
                new ArmCommand(m_armSubsystem, ArmConstants.kArmLowPosition),     
                new DriveDistanceCommand(m_driveTrain, 0.30, (-36.0), m_GyroSubsystem, m_encoderSubsystem)  // Back up into the field.           
                // new DriveByAngleCommand(m_driveTrain, m_GyroSubsystem, 0.2, 180.0 - 10.0)                    // Turn to face the field.
              )
          ));
        }
  }
}
