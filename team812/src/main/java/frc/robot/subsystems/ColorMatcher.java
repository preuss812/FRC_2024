/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.Constants.ColorConstants;

public class ColorMatcher extends SubsystemBase {
  /**
   * Creates a new ColorMatcher.
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  // Team812 calibrated color definitions, see Constants.java
  public final Color kBlueTarget = ColorMatch.makeColor(
    ColorConstants.kBlueTargetRBG[0], 
    ColorConstants.kBlueTargetRBG[1],
    ColorConstants.kBlueTargetRBG[2]
  );
  public final Color kGreenTarget = ColorMatch.makeColor(
    ColorConstants.kGreenTargetRGB[0],
    ColorConstants.kGreenTargetRGB[1],
    ColorConstants.kGreenTargetRGB[2]
  );
  public final Color kRedTarget = ColorMatch.makeColor(
    ColorConstants.kRedTargetRGB[0],
    ColorConstants.kRedTargetRGB[1],
    ColorConstants.kRedTargetRGB[2]
  );
  public final Color kYellowTarget = ColorMatch.makeColor(
    ColorConstants.kYellowTargetRGB[0],
    ColorConstants.kYellowTargetRGB[1],
    ColorConstants.kYellowTargetRGB[2]
  );


  public ColorMatcher() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.setConfidenceThreshold(ColorConstants.kColorConfidenceThreshhold);
  }

  public Color get_color() {
    final Color detectedColor = m_colorSensor.getColor();
    final int proximity = m_colorSensor.getProximity();

    String colorString;
    final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (proximity >= 100) 
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    } else {
      colorString = "Out of range";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("Confidence", match.confidence);  
    SmartDashboard.putString("Detected Color", colorString);

    return match.color;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean isRed(Color color) {
    return (color == kRedTarget);
  }
  public boolean isBlue(Color color) {
    return (color == kBlueTarget);
  }
  public boolean isYellow(Color color) {
    return (color == kYellowTarget);
  }
  public boolean isGreen(Color color) {
    return (color == kGreenTarget);
  }
}
