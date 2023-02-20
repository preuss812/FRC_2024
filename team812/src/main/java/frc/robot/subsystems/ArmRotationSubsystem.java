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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PCMConstants;

public class ArmRotationSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_arm = new WPI_TalonSRX(CANConstants.kArmMotor);
  private static boolean hasBeenHomed = true;

  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
      CANConstants.kPCM,
      PneumaticsModuleType.CTREPCM,
      PCMConstants.kArmExtension[0],
      PCMConstants.kArmExtension[1]);

  /** Creates a new ArmSubsystem. */
  public ArmRotationSubsystem() {
    m_arm.configFactoryDefault();
    m_arm.setNeutralMode(NeutralMode.Brake);

    // This is a CLOSED loop system. Do not uncomment or enable
    // OpenloopRamp for the PID controlled arm.
    // m_arm.configOpenloopRamp(PidConstants.xxx_kArm_rampRate_xxx);

    // Configure the feedback sensor with the type (QuadEncoder),
    // the PID identifier within the Talon (pid 0) and the timeout (50ms)
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);

    // Invert motor (setInverted) so that the Talon LEDs are green when driving
    // forward (up)
    // Phase sensor should have a positive increment as the Talon drives the arm up
    m_arm.setInverted(false);
    m_arm.setSensorPhase(true); // Attempts to make it positive

    // Set status frame period to 10ms with a timeout of 10ms
    // 10 sets timeouts for Motion Magic
    // 13 sets timeouts for PID 0
    m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

    // Configure low and high output levels to help remove any
    // stalling that might occur where stalling means that power is being
    // applied, but the motor isn't moving due to friction or inertia. This
    // can help the motor not burn itself out.
    m_arm.configNominalOutputForward(0, 10);
    m_arm.configNominalOutputReverse(0, 10);
    m_arm.configPeakOutputForward(0.4, 10);
    m_arm.configPeakOutputReverse(-0.2, 10);

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

    // Make sure the forward and reverse limit switches are enabled and configured
    // normally open
    m_arm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_arm.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

  }

  private final int incrementSize = 50;

  public void rotate(double position) {
    double absolutePosition = getPosition();
    // if the joystick is nearly centered, ignore it
    if (Math.abs(position) < 0.01) {
      return;
    }

    absolutePosition += position * incrementSize;
    SmartDashboard.putNumber("rotate pos", absolutePosition);

    if (absolutePosition > ArmConstants.kArmBallGathering
        && absolutePosition < ArmConstants.kArmScorePosition) {
      setPosition(absolutePosition);
    }
  };


  public void rotate2(double speed) {
    double l_speed = speed;
    double l_position = getPosition();
    String path;

    if (!isHome()) {
      l_speed = 0.0;
      path = "not homed";
    } else if (l_speed > 0.0) {
      if (l_position >= ArmConstants.kArmTopPosition) {
        l_speed = 0.0;
        path = "endgame";
      } else if (l_position >= ArmConstants.kArmScorePosition) {
        l_speed = 0.0;
        path = "not end game";
      } else {
        path = "speed > 0 no change";
      }
    } else if (l_speed < 0.0) {
      if (l_position <= ArmConstants.kArmBallGathering) {
        l_speed = 0.0;
        path = "speed < 0 limited";
      } else {
        path = "speed < 0 no change";
      }
    } else {
      path = "lspeed = 0";
      // l_speed = 0.0;
    }

    SmartDashboard.putNumber("r2_speed", speed);
    SmartDashboard.putNumber("r2_l_speed", l_speed);
    SmartDashboard.putNumber("r2_l_position", l_position);
    SmartDashboard.putString("rotate2_path", path);


    m_arm.set(ControlMode.PercentOutput, l_speed);
    // m_arm.set(ControlMode.Velocity, l_speed, DemandType.Neutral, demand1);
  }


  public void test_rotate(double speed) {
    double l_speed = speed;
    double l_position = getPosition();

    l_speed = MathUtil.clamp(l_speed, -0.30, 0.10);

    SmartDashboard.putNumber("test_rotate_l_speed", l_speed);
    SmartDashboard.putNumber("test_rotate_l_position", l_position);

    m_arm.set(ControlMode.PercentOutput, l_speed);
    // m_arm.set(ControlMode.Velocity, l_speed, DemandType.Neutral, demand1);
  }

  public double setPosition(double position) {
    if (isHome() && position >= ArmConstants.kArmBallGathering) {
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
    return position;
  }

  public void setSensorPosition(double position) {
    m_arm.setSelectedSensorPosition(position, 0, 10);
  }

  public void setSensorReference() {
    double l_position = ArmConstants.kArmReferencePosition;
    m_arm.setSelectedSensorPosition(l_position, 0, 10);
    setHomePosition(l_position);
    setHome();
  }

  public boolean isTopLimitSwitchClosed() {
    return (m_arm.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  public boolean isBottomLimitSwitchClosed() {
    return (m_arm.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  public void armExtend() {
    // check the height to be sure that we don't extend the arms
    // into the wheels.
    if (getPosition() >= (ArmConstants.kArmBallGathering - ArmConstants.kArmThreshold) && isHome()) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      SmartDashboard.putString("Armsolenoid", "extended");
    }
  }

  public void armRetract() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putString("Armsolenoid", "retracted");
  }

  public void setHome() {
    hasBeenHomed = true;
    System.out.println("setHome hasBeenHomed: " + hasBeenHomed);
  }

  public void unsetHome() {
    hasBeenHomed = false;
    // System.out.println("unsetHome hasBeenHomed: " + hasBeenHomed);
  }

  public void unsetHome(String msg) {
    hasBeenHomed = false;
    // System.out.println("unsetHome called from >" + msg + "< and hasBeenHomed: " +
    // hasBeenHomed);
  }

  public boolean isHome() {
    return hasBeenHomed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm pos:", getPosition());
    SmartDashboard.putBoolean("Arm Homed?", isHome());
    SmartDashboard.putBoolean("ARM topsw closed", isTopLimitSwitchClosed());
    SmartDashboard.putBoolean("ARM botsw closed",isBottomLimitSwitchClosed());
  }
}
