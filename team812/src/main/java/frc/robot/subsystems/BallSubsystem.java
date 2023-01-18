/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

/**
 * Add your docs here.
 */

public class BallSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final WPI_TalonSRX m_intake = new WPI_TalonSRX(CANConstants.kIntakeMotor);

  public BallSubsystem() {
    stop();

    m_intake.configFactoryDefault();
    m_intake.setNeutralMode(NeutralMode.Brake);
  }
  
  public void intake() {
    //   double speed = Robot.nttable.getEntry("ypotvalue").getDouble(1.0);
       double speed = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotX, 0.0, 1.0);
       m_intake.set(-speed);
       SmartDashboard.putNumber("BallSubsystem intake speed", speed);
     }
   
  public void outtake() {
      double speed = RobotContainer.m_BlackBox.getPotValueScaled(OIConstants.kControlBoxPotY, 0.0, 1.0);
       m_intake.set(speed);
       SmartDashboard.putNumber("BallSubsystem output speed", speed);
     }
   

/*
   @Override
    public void initDefaultCommand() {
     // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
   }*/
  public void stop() {
    m_intake.set(0.0);
  }
}