/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PCMConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Robot;

public class SpinTheWheelSubsystem extends SubsystemBase {

  private final WPI_TalonSRX m_SpinTheWheelSubsystem = new WPI_TalonSRX(CANConstants.kSpinMotor);
  public SpinTheWheelSubsystem() {
    stop();
    m_SpinTheWheelSubsystem.configFactoryDefault();
    m_SpinTheWheelSubsystem.setNeutralMode(NeutralMode.Brake);
  }

    public void forward(double speed) {
      m_SpinTheWheelSubsystem.set(speed);

    }
     
    public void reverse(double speed) {
      m_SpinTheWheelSubsystem.set(-speed);
    }
    
    public void stop() {
      m_SpinTheWheelSubsystem.set(0.0);
    }	
  }
  
