// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmExtensionConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class ArmExtensionSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_armExtension = new WPI_TalonSRX(CANConstants.kArmExtensionMotor);
  private static boolean hasBeenHomed = false;
  private static double targetPosition = 0.0;
  
  /** Creates a new ArmExtensionSubsystem. */
  public ArmExtensionSubsystem() {
    m_armExtension.configFactoryDefault();
    m_armExtension.setNeutralMode(NeutralMode.Brake);

    // This is a CLOSED loop system. Do not uncomment or enable
    // OpenloopRamp for the PID controlled arm.
    // m_armExtension.configOpenloopRamp(PidConstants.xxx_kArmExtension_rampRate_xxx);

    // Configure the feedback sensor with the type (QuadEncoder),
    // the PID identifier within the Talon (pid 0) and the timeout (50ms)
    m_armExtension.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);

    // Invert motor (setInverted) so that the Talon LEDs are green when driving
    // forward (up)
    // Phase sensor should have a positive increment as the Talon drives the arm up
    m_armExtension.setInverted(true);
    m_armExtension.setSensorPhase(false); // Attempts to make it positive

    // Set status frame period to 10ms with a timeout of 10ms
    // 10 sets timeouts for Motion Magic
    // 13 sets timeouts for PID 0
    m_armExtension.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    m_armExtension.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

    // Configure low and high output levels to help remove any
    // stalling that might occur where stalling means that power is being
    // applied, but the motor isn't moving due to friction or inertia. This
    // can help the motor not burn itself out.
    m_armExtension.configNominalOutputForward(0, 10);
    m_armExtension.configNominalOutputReverse(0, 10);
    m_armExtension.configPeakOutputForward(0.5, 10);
    m_armExtension.configPeakOutputReverse(-0.3, 10);

    // Configure the Motion Magic parameters for PID 0 within the Talon
    // The values for P, I, D, and F will need to be determined emperically
    m_armExtension.selectProfileSlot(0, 0);
    m_armExtension.config_kP(0, PidConstants.kArmExtension_kP, 10);
    m_armExtension.config_kI(0, PidConstants.kArmExtension_kI, 10);
    m_armExtension.config_kD(0, PidConstants.kArmExtension_kD, 10);
    m_armExtension.config_kF(0, PidConstants.kArmExtension_kF, 10);

    // Velocity in sensor units per 100ms
    m_armExtension.configMotionCruiseVelocity(150.0, 10);
    // Acceleration in sensor units per 100ms per second
    m_armExtension.configMotionAcceleration(150.0, 10);

    // Make sure the forward and reverse limit switches are enabled and configured
    // normally open
    m_armExtension.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        0);
    m_armExtension.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
        0);
    m_armExtension.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

  }

  /*
  public void reposition(double speed) {
    double l_speed = speed;
    double l_position = getPosition();
    String path;

    if (!isHome()) {
      l_speed = 0.0;
      path = "not homed";
    } else if (l_speed > 0.0) {
      if (l_position >= ArmExtensionConstants.kArmExtensionFullyExtendedPosition) {
        l_speed = 0.0;
        path = "already fully extended";
      } else {
        path = "speed > 0 no change";
      }
    } else if (l_speed < 0.0) {
      if (l_position <= ArmExtensionConstants.kArmExtensionFullyRetractedPosition) {
        l_speed = 0.0;
        path = "already fully retracted";
      } else {
        path = "speed < 0 no change";
      }
    } else {
      path = "lspeed = 0";
      // l_speed = 0.0;
    }
    SmartDashboard.putNumber("ArmExtension:r2_speed", speed);
    SmartDashboard.putNumber("ArmExtension:r2_l_speed", l_speed);
    SmartDashboard.putNumber("ArmExtension:r2_l_position", l_position);
    SmartDashboard.putString("ArmExtension:rotate2_path", path);

    m_armExtension.set(ControlMode.PercentOutput, l_speed);
    // m_armExtension.set(ControlMode.Velocity, l_speed, DemandType.Neutral,
    // demand1);
  }
  */
// TODO Understand why this works with PID Controller - dph 2023-03-01
  public void testMoveInOut(double speed) {
    double l_speed = speed;
    l_speed = MathUtil.clamp(l_speed, -0.50, 0.50); // TODO increase speed - dph 2023-03-01
    SmartDashboard.putNumber("ArmExtension test speed", l_speed);
    double newTarget = targetPosition + Math.signum(l_speed)*200;
    // m_armExtension.set(ControlMode.Position, targetPosition);
    setPosition(newTarget);
  }

  public double setPosition(double position) {
    position = MathUtil.clamp(position, ArmExtensionConstants.kArmExtensionMinPosition, ArmExtensionConstants.kArmExtensionMaxPosition);
    
    if (isHome()) {
      m_armExtension.set(ControlMode.Position, position);
      targetPosition = position;
      SmartDashboard.putNumber("ArmExtensionSubPos", position);
    }
    return getPosition();
  }

  // Only to be used when homing the robot
  public double setHomePosition(double position) {
    // m_armExtension.set(ControlMode.Position, position); // TODO: fix
    return getPosition();
  }

  public double getPosition() {
    double position = m_armExtension.getSelectedSensorPosition(0);
    return position;
  }

  public double getTargetPosition() {
    return targetPosition;
  }
  
  public void setSensorPosition(double position) {
    m_armExtension.setSelectedSensorPosition(position, 0, 10);
  }

  public void setSensorReference() {
    double l_position = ArmExtensionConstants.kArmExtensionReferencePosition;
    m_armExtension.setSelectedSensorPosition(l_position, 0, 10);
    setHomePosition(l_position);
    setHome();
  }

  public boolean isInLimitSwitchClosed() {
    return (m_armExtension.isRevLimitSwitchClosed() == 1 ? true : false); // TODO verify swap of in/out - dph 2023-03-01
  }

  public boolean isOutLimitSwitchClosed() {
    return (m_armExtension.isFwdLimitSwitchClosed() == 1 ? true : false);  // TODO verify swap of in/out - dph 2023-03-01
  }

  public void setHome() {
    hasBeenHomed = true;
  }

  public void unsetHome() {
    hasBeenHomed = false;
  }

  public void unsetHome(String msg) {
    hasBeenHomed = false;
  }

  public boolean isHome() {
    return hasBeenHomed;
  }

  // Takes rotation in ticks and returns extension in ticks
  private double getExtensionBoundary(double rotationInTicks){
    // Transform rotation in ticks into degrees relative to horizontal arm.
    double theta = (rotationInTicks - ArmConstants.kArmHorizontalRotationPosition)*ArmConstants.kArmDegreesPerTick;
    SmartDashboard.putNumber("ArmExtension theta", theta);
    // Find the total allowed length of the arm from the pivot point
    double maxLengthInMeters = ArmExtensionConstants.kArmExtensionHorizontalExtensionPosition/Math.cos(Units.degreesToRadians(theta));
    // Subtract the fixed length of the arm from the total length to get the extension in meters and convert that to ticks.
    double maxExtensionPosition = (maxLengthInMeters - ArmExtensionConstants.kArmExtensionRetractedLength)*ArmExtensionConstants.kArmExtensionTicksPerMeter;
    maxExtensionPosition = MathUtil.clamp(maxExtensionPosition, ArmExtensionConstants.kArmExtensionMinPosition, ArmExtensionConstants.kArmExtensionMaxPosition);
    // If we are below horizontal perform an additional check for the arm extending below the plane of the ground.
    if (theta < 0) {
      double lowerLimit = ArmExtensionConstants.kArmExtensionPivotToWheelsOnFloorLine; // Debug only, remove.
      double minArmLength = ArmExtensionConstants.kArmExtensionRetractedLength;
      double maxArmLengthAtThisAngle = -ArmExtensionConstants.kArmExtensionPivotToWheelsOnFloorLine/Math.sin(Units.degreesToRadians(-theta));
      double maxArmExtensionLength = maxArmLengthAtThisAngle - minArmLength;
      if (maxArmExtensionLength < 0.0) maxArmExtensionLength = 0.0;
      double maxExtensionPositionForFloor = maxArmExtensionLength*ArmExtensionConstants.kArmExtensionTicksPerMeter;
      if ( maxExtensionPositionForFloor< maxExtensionPosition)
        maxExtensionPosition = maxExtensionPositionForFloor;
    }
      return maxExtensionPosition; // TODO: fix, this value is too small and I don't know what constant it is using for boundary
  }

  private double maxExtensionForRotation() {
    double currentRotation = RobotContainer.m_ArmRotationSubsystem.getTargetPosition();
    double rotationInTicks = getPosition();

    getExtensionBoundary(rotationInTicks);
    double targetExtension = getTargetPosition();
    double currentExtension = getPosition();
    double adjustedExtension = MathUtil.clamp(getExtensionBoundary(currentRotation), ArmExtensionConstants.kArmExtensionMinPosition, targetExtension);

    return adjustedExtension;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double adjustedExtension = maxExtensionForRotation();
    double targetExtension = getTargetPosition();

    // Prevent the rotation of the arm from taking the arm out of the legal perimiter of the robot.
    SmartDashboard.putNumber("Extension Limit", adjustedExtension);

    if (adjustedExtension != targetExtension){
     setPosition(adjustedExtension); // Note this corrupts the target position in some cases. TODO remember the true goal
      // m_armExtension.set(ControlMode.Position, adjustedExtension);

    }

    SmartDashboard.putNumber("ArmExtension pos", getPosition());
    // SmartDashboard.putNumber("ArmExtension target", m_armExtension.getClosedLoopTarget()); // This was generating a warning
    SmartDashboard.putNumber("ArmExtension target", getTargetPosition());
    SmartDashboard.putBoolean("ArmExtension Homed?", isHome());
    SmartDashboard.putBoolean("ArmExtension outsw", isOutLimitSwitchClosed());
    SmartDashboard.putBoolean("ArmExtension insw", isInLimitSwitchClosed());

    // Resets the set position to 0 when the limit switch is switched
    if (isInLimitSwitchClosed()) {
      setSensorReference();
    }
    if (isOutLimitSwitchClosed()) {
      setSensorPosition(ArmExtensionConstants.kArmExtensionMaxPosition);
   }
  }
}
