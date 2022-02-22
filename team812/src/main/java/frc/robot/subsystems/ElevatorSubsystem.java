/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final WPI_TalonSRX m_elevatorLeft = new WPI_TalonSRX(CANConstants.kElevatorMotorLeft);
  private final WPI_TalonSRX m_elevatorRight = new WPI_TalonSRX(CANConstants.kElevatorMotorRight);

  private WPI_TalonSRX m_elevator = m_elevatorLeft;

  public ElevatorSubsystem() {
    stop();
    m_elevatorLeft.configFactoryDefault();
    m_elevatorLeft.setNeutralMode(NeutralMode.Brake);
    m_elevatorLeft.setInverted(true);

    m_elevatorRight.configFactoryDefault();
    m_elevatorRight.setNeutralMode(NeutralMode.Brake);
    m_elevatorRight.setInverted(true);

    m_elevatorRight.follow(m_elevatorLeft);
  }
  
  public void up() {
       double speed = 0.5;
       m_elevator.set(speed);
     }
   
  public void down() {
       double speed= 0.5;
       m_elevator.set(-speed);
     }

/*
   @Override
    public void initDefaultCommand() {
     // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
   }*/
  public void stop() {
    m_elevator.set(0.0);
  }
}
