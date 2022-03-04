/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PCMConstants;

/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final WPI_TalonSRX m_elevatorLeft = new WPI_TalonSRX(CANConstants.kElevatorMotorLeft);
  private final WPI_TalonSRX m_elevatorRight = new WPI_TalonSRX(CANConstants.kElevatorMotorRight);

  private WPI_TalonSRX m_elevator = m_elevatorLeft;
  private static Boolean end_game = false;

  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
    CANConstants.kPCM,
    PneumaticsModuleType.CTREPCM,
    PCMConstants.kBarHooks[0],
    PCMConstants.kBarHooks[1]
  );

  public ElevatorSubsystem() {
    stop();
    m_elevatorLeft.configFactoryDefault();
    m_elevatorLeft.setNeutralMode(NeutralMode.Brake);
    m_elevatorLeft.setInverted(false);

    m_elevatorRight.configFactoryDefault();
    m_elevatorRight.setNeutralMode(NeutralMode.Brake);
    m_elevatorRight.setInverted(false);

    m_elevatorRight.follow(m_elevatorLeft);

    end_game = false;
  }
  
  public void up() {
       double speed = 0.2;
       if( end_game )
         m_elevator.set(speed);
     }
   
  public void down() {
       double speed= 0.90;
       if( end_game )
        m_elevator.set(-speed);
     }

     public void openGrip() {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      SmartDashboard.putString("ElevatorGrip", "open");
    }
    public void closeGrip() {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      SmartDashboard.putString("ElevatorGrip", "closed");
    }

    public void elevate(double throttle) {
      if( end_game )
        m_elevator.set(-throttle);
      }
    
    public void enable_elevator() {
      end_game = true;
    }

    public boolean is_endgame() {
      return end_game;
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
