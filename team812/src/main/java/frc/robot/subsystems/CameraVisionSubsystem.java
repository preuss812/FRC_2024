// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class CameraVisionSubsystem extends SubsystemBase {
  /** Creates a new CameraVisionSubsystem. */
  public PhotonCamera camera = new PhotonCamera("pv-812");
  
  public CameraVisionSubsystem() {
    //camera.setDriverMode(true); // a guess
    //camera.setPipelineIndex(0);
  }

  public boolean hasTargets ()
  {
    var result = camera.getLatestResult();
    var idx = camera.getPipelineIndex();
    SmartDashboard.putNumber("pipeline",idx);
  //camera.setDriverMode(false); // a test if this does what we expect

    if (result.hasTargets()) {
      SmartDashboard.putString("Visual Target", "Yes");
    } else {
      SmartDashboard.putString("Visual Target", "No");
    }
    return result.hasTargets();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
