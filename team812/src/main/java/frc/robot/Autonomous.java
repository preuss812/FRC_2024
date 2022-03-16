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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BlackBoxSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

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
    blackBox.readBits();

    addCommands(
      new SequentialCommandGroup(
        new ArmHomeCommand(m_armSubsystem),
        new DriveBackwardCommand(m_driveTrain, 0.5, -0.2).withTimeout(2.0)
      )
    );
  }
}
