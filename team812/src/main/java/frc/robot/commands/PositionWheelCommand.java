/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorMatcher;
import frc.robot.subsystems.SpinTheWheelSubsystem;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.PositionWheelConstants;

public class PositionWheelCommand extends CommandBase {
  /**
   * Creates a new PositionWheel.
   */
    private final SpinTheWheelSubsystem m_SpinTheWheelSubsystem;
    private final ColorMatcher m_ColorMatcher;
    private Color initialColor;
    private int   initialColorId;
    private Color lastColor;
    private int lastColorIdDetected;
    private int goalColorId;
    private int goalColorIdAdjusted;
    private int samplesInThisSlice;

  public PositionWheelCommand(SpinTheWheelSubsystem motorSubsystem, ColorMatcher sensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_SpinTheWheelSubsystem = motorSubsystem;
      addRequirements(motorSubsystem);
      m_ColorMatcher = sensorSubsystem;
      addRequirements(sensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      goalColorId = m_ColorMatcher.getFMScolorId();  //ColorConstants.kColorGreen;  // Need to get from the MCP :-)
      goalColorIdAdjusted = (goalColorId+2) % 4;  // Advance 2 colors to account for the 90 degree offset of our robot from the mechanism's color sensor 
      System.out.printf("We are looking for color = %s which means our color matcher needs to see %s\n", ColorConstants.kColorNames[goalColorId], ColorConstants.kColorNames[goalColorIdAdjusted]);
      initialColorId = m_ColorMatcher.getColorIdCorrected(ColorConstants.kColorUnknown);
      if (initialColorId == goalColorIdAdjusted) {
          System.out.printf("That is surprising.  We are apparently already on the goal color id.\n");
      }
      lastColor = initialColor;
      samplesInThisSlice = 0;
      lastColorIdDetected = initialColorId;
      System.out.println("is initialized*");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Color detectedColor = m_ColorMatcher.get_color();
    int detectedColorId = m_ColorMatcher.getColorIdCorrected(lastColorIdDetected);
    System.out.printf("lastColorId=%d detected=%d\n",lastColorIdDetected,detectedColorId);
     if (initialColorId == ColorConstants.kColorUnknown) {
       if (detectedColorId != ColorConstants.kColorUnknown) {
         System.out.printf("Found initial color\n");
       
        initialColorId = detectedColorId;
        // lastColorIdDetected = detectedColorId;
       }
     } else if (detectedColorId != ColorConstants.kColorUnknown) {
       if (detectedColorId != lastColorIdDetected)
       {
         samplesInThisSlice = 1;
         // We moved to the next color.
         if (detectedColorId == goalColorIdAdjusted) {
            System.out.printf("Entered the goal color slice\n");
          } else {
            System.out.printf("Entered a non-goal color slice\n");
          }
       } else {
          samplesInThisSlice++;
          System.out.printf("Continuing in the current slice. Samples=%d\n",samplesInThisSlice);
           } 
     } else {
        // We went from a known color to unknown.  That happens when we see red -> blue or blue -> red.
        // this will likely result in a bad rotation count and cause the process to time out.
        System.out.printf("We lost the color\n");
     }
     lastColorIdDetected = detectedColorId;
    /*
    print_color(detectedColor);
    */
    m_SpinTheWheelSubsystem.forward(PositionWheelConstants.kPositionWheelMotorSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("is finished+");
      if( lastColorIdDetected == goalColorIdAdjusted && samplesInThisSlice >= PositionWheelConstants.kExpectedSamplesPerSlice/2) {
        System.out.println("PositionWheel centered in target slice.\n");
    	m_SpinTheWheelSubsystem.forward(0.0);
        return true;
      }
      else
      {
        return false;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("is ending///////////");
    m_SpinTheWheelSubsystem.stop();
  }

  public void print_color(Color color) {
    String colorString;

    if (m_ColorMatcher.isBlue(color)) {
      colorString = "Blue";
    } else if (m_ColorMatcher.isGreen(color)) {
      colorString = "Green";
    } else if (m_ColorMatcher.isRed(color)) {
      colorString = "Red";
    } else if (m_ColorMatcher.isYellow(color)) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    } 
    System.out.printf("Color detected: %s\n", colorString);
  }
   
}
