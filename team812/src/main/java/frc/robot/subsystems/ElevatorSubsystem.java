/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


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

  private final WPI_TalonSRX m_elevator = new WPI_TalonSRX(CANConstants.kElevatorMotor);
  public ElevatorSubsystem() {
    stop();
    m_elevator.configFactoryDefault();
    m_elevator.setNeutralMode(NeutralMode.Brake);
  }
  
  public void up() {
       //double speed = Robot.nttable.getEntry("ypotvalue").getDouble(1.0);
       //double speed = RobotContainer.m_BlackBox.getPotValueScaled(Constants.OIConstants.kControlBoxPotY, 0.0, 1.0);
       double speed = 1.0;
       m_elevator.set(speed);
       //SmartDashboard.putNumber("forward speed", speed);
     }
   
  public void down() {
      // double speed = RobotContainer.m_BlackBox.getPotValueScaled(Constants.OIConstants.kControlBoxPotY, 0.0, 1.0);
       double speed= 1.0;
       m_elevator.set(-speed);
       //SmartDashboard.putNumber("reversed speed", -speed);
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
