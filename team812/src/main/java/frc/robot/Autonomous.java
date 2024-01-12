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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmExtensionConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.BlackBoxSubsystem;

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
    boolean balancing = true;
    boolean readingBlackBoxSwitch = false;

    m_driveTrain = subsystem;

    System.out.printf("*** Entering Autonomous mode\n");

    BlackBoxSubsystem blackBox = RobotContainer.m_BlackBox;
   
  }
}
