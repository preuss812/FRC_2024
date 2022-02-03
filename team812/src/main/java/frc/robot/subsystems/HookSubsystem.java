/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants;

public class HookSubsystem extends SubsystemBase {
  /**
   * Creates a new HookSubsystem.
   */
  private final WPI_VictorSPX m_hook = new WPI_VictorSPX(CANConstants.kHookMotor);

  public HookSubsystem() {
    stop();
    m_hook.configFactoryDefault();
    m_hook.setNeutralMode(NeutralMode.Brake);
  }

  public void up() {
 //   double speed = Robot.nttable.getEntry("ypotvalue").getDouble(1.0);
    double speed = RobotContainer.m_BlackBox.getPotValueScaled(Constants.OIConstants.kControlBoxPotY, 0.0, 1.0);
    m_hook.set(speed);
    SmartDashboard.putNumber("forward speed", speed);
  }

  public void down() {
    double speed = RobotContainer.m_BlackBox.getPotValueScaled(Constants.OIConstants.kControlBoxPotY, 0.0, 1.0);
    m_hook.set(-speed);
    SmartDashboard.putNumber("reversed speed", -speed);
  }

  public void stop() {
    m_hook.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
