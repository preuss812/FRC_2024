// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.CameraVisionSubsystem;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestProfiledPIDCommand extends ProfiledPIDCommand {
  /** Creates a new TestProfiledPIDCommand. */
  private final CameraVisionSubsystem m_camera;
  private final DriveTrain m_driveTrain;
  private double lastYawError = 0;
  // private final Joystick m_joystick;
  public TestProfiledPIDCommand(CameraVisionSubsystem camera, DriveTrain driveTrain) { //}, Joystick joystick) {

    
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.02, // Determined emperically with TurnRightBB
            0.0002,
            0.002,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(90,30)),  // These are degrees/second and degrees/second/second
        // This should return the measurement
        () -> 0,
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
        });
      m_controller.setIntegratorRange(-0.3, 0.3); // Limit the accumulated error in the Integral term of the PID  Values??? - dph
      m_camera = camera;
      m_driveTrain = driveTrain;
      // m_joystick = joystick;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(camera, driveTrain);

    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
    /* 
    if (m_joystick.getRawButtonPressed(2)) {
      m_controller.setGoal(30.0); // 30 degree angle relative to the target
    } else if (m_joystick.getRawButtonPressed(3)) {
      m_controller.setGoal(-30.0); // -30 degree angle relative to the target
    }
    */
    m_controller.setGoal(0); // debug for dph
    PhotonTrackedTarget target;
    double rotation = 0.0;
    double throttle = 0.00314;
    double yaw = 0.0;
    double yawError = 0.0;
    // Should look into supplying function getMeasurement - dph
    target = m_camera.getBestTarget();
    // If no apriltag found, stop.
    if (target == null) {
        SmartDashboard.putString("TargetFound", "No");
    } else {
      SmartDashboard.putString("TargetFound", "Yes");

      yaw = target.getYaw();
      SmartDashboard.putNumber("TargetRotationYaw", yaw);

      rotation = m_controller.calculate(yaw);
      rotation = MathUtil.clamp(rotation,-0.5,0.5);
      rotation = rotation + Math.signum(rotation) * 0.2;
      yawError = m_controller.getPositionError();
      // if the sign of the error switches, we have passed the target. reset the pid controller.
      boolean disableReset = true;
      if (!disableReset && Math.signum(yawError) != Math.signum(lastYawError))  // Disabled at the moment.
      {
        m_controller.reset(20);
        rotation = 0.0; // Could be abrupt, also a hack.  possible recall calculate after the reset
      }
      lastYawError = yawError;
    }
    SmartDashboard.putNumber("TargetRotationSpeed", rotation);
    SmartDashboard.putNumber("TargetThrottle", throttle);

    // Run controller and update motor output
    m_driveTrain.arcadeDrive(throttle, rotation); // preussDrive was scaling the results.  Wanted to avoid that for how.
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}