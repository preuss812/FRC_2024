// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PCMConstants;

public class ArmSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_arm = new WPI_TalonSRX(CANConstants.kArmMotor);
  private static boolean hasBeenHomed = false;

  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
    CANConstants.kPCM,
    PneumaticsModuleType.CTREPCM,
    PCMConstants.kArmExtension[0],
    PCMConstants.kArmExtension[1]
  );

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_arm.configFactoryDefault();
    m_arm.setNeutralMode(NeutralMode.Brake);
    m_arm.configOpenloopRamp(PidConstants.kArm_rampRate);

    // Configure the feedback sensor with the type (QuadEncoder), 
    // the PID identifier within the Talon (pid 0) and the timeout (50ms)
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);

    // Invert motor (setInverted) so that the Talon LEDs are green when driving forward (up)
    // Phase sensor should have a positive increment as the Talon drives the arm up
    m_arm.setInverted(true);
    m_arm.setSensorPhase(false); //Attempts to make it positive

    // Set status frame period to 10ms with a timeout of 10ms
    // 10 sets timeouts for Motion Magic
    // 13 sets timeouts for PID 0
    m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

    // Configure low and high output levels to help remove any
    // stalling that might occur where stalling means that power is being
    // applied, but the motor isn't moving due to friction or inertia. This
    // can help the motor not burn itself out.
    m_arm.configNominalOutputForward(0,10);
    m_arm.configNominalOutputReverse(0, 10);
    m_arm.configPeakOutputForward(1.0,10);
    m_arm.configPeakOutputReverse(-1.0, 10);

    // Configure the Motion Magic parameters for PID 0 within the Talon
    // The values for P, I, D, and F will need to be determined emperically
    m_arm.selectProfileSlot(0, 0);
    m_arm.config_kP(0, PidConstants.kArm_kP, 10);
    m_arm.config_kI(0, PidConstants.kArm_kI, 10);
    m_arm.config_kD(0, PidConstants.kArm_kD, 10);
    m_arm.config_kF(0, PidConstants.kArm_kF, 10);

    // Velocity in sensor units per 100ms
    m_arm.configMotionCruiseVelocity(150.0, 10);
    // Acceleration in sensor units per 100ms per second
    m_arm.configMotionAcceleration(150.0, 10);

    // Make sure the forward and reverse limit switches are enabled and configured normally open
    m_arm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,0);
    m_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,0);
    m_arm.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0,0,0,0); 

  }
   private double absolutePosition = getPosition();

  public void rotate(double position) {
//    System.out.println(absolutePosition);
    SmartDashboard.putNumber("rotate pos", absolutePosition);
    absolutePosition=absolutePosition+position;
    if (absolutePosition > ArmConstants.kArmBallGathering && absolutePosition < ArmConstants.kArmScorePosition) {
      setPosition(absolutePosition);
   }
  };
  public double setPosition(double position)  {
    if( hasBeenHomed && position >= ArmConstants.kArmBallGathering ) {
      m_arm.set(ControlMode.Position, position);
      SmartDashboard.putNumber("ArmSubPos", position);
    }
    return getPosition();
  }

  // Only to be used when homing the robot
  public double setHomePosition(double position) {
    m_arm.set(ControlMode.Position, position);
    return getPosition();
  }

  public double getPosition() {
    double position = m_arm.getSelectedSensorPosition(0);
    SmartDashboard.putNumber("Arm pos:", position);
    return position;
  }

  public void setSensorPosition(double position) {
    m_arm.setSelectedSensorPosition(position, 0, 10);
  }


  public boolean isTopLimitSwitchClosed () {
    return (m_arm.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  public boolean isBottomLimitSwitchClosed() {
    return (m_arm.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  public void armExtend() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    SmartDashboard.putString("Armsolenoid", "extended");
  }
  public void armRetract() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Armsolenoid", "retracted");
  }

  public void setHome() {
    hasBeenHomed = true;
  }

  public void unsetHome() {
    hasBeenHomed = false;
  }

  public boolean isHome() {
    return hasBeenHomed;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getPosition();
  }
}
