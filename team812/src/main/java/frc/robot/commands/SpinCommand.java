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
import frc.robot.Constants.SpinConstants;

public class SpinCommand extends CommandBase {
  /**
   * Creates a new SpinCommand.
   */
    private int rotationCount;
    private final SpinTheWheelSubsystem m_SpinTheWheelSubsystem;
    private final ColorMatcher m_ColorMatcher;
    private boolean wasItRed;
    private Color initialColor;
    private int   initialColorId;
    private int colorCounter;
    private Color lastColor;
    private int lastColorIdDetected;

  public SpinCommand(SpinTheWheelSubsystem motorSubsystem, ColorMatcher sensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_SpinTheWheelSubsystem = motorSubsystem;
      addRequirements(motorSubsystem);
      m_ColorMatcher = sensorSubsystem;
      addRequirements(sensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      rotationCount = 0;
      wasItRed = false;
      initialColorId = m_ColorMatcher.getColorIdCorrected(ColorConstants.kColorUnknown);
      lastColor = initialColor;
      colorCounter = 0;
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
         // We moved to the next color.
         if (detectedColorId == initialColorId) {
            rotationCount++;
            colorCounter = 1;
            //System.out.printf("I rotated********* Rotations=%d Samples=%d\n", rotationCount, colorCounter);
          }
        } else {
            colorCounter++;
            //System.out.printf("I sampled********* Rotations=%d Samples=%d\n", rotationCount, colorCounter);
        } 
     } else {
        // We went from a known color to unknown.  That happens when we see red -> blue or blue -> red.
        // this will likely result in a bad rotation count and cause the process to time out.
        System.out.printf("We lost the color\n");
     }
     lastColorIdDetected = detectedColorId;
    /*
    if(wasItRed == false) {
        rotationCount++;
        
        
        System.out.printf("Color Counter **** %d\n", colorCounter);
        colorCounter = 0;

      }
      wasItRed = true;
    } else {
      wasItRed = false;
      colorCounter++;
    }
    print_color(detectedColor);
    */
    /*
    if(m_ColorMatcher.isRed(detectedColor)) {
     // System.out.println("I see red!");
      if(wasItRed == false) {
        rotationCount = rotationCount + 1;
        System.out.printf("I rotated********* %d\n", rotationCount);
        
      }
      wasItRed = true;
    } else {
     // System.out.println("Nope!");
      wasItRed = false;
    }
    */

    m_SpinTheWheelSubsystem.forward(SpinConstants.kSpinMotorSpeed);
    //System.out.println("is executed-");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("is finished+");
      if( rotationCount >= SpinConstants.kColorRotationCountMax ) {
        System.out.println("is finished+++++++");
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
