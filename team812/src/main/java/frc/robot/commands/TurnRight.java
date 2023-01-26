package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.DriveTrain;

public class TurnRight extends CommandBase {
    // private void tankDrive(double leftSpeed, double rightSpeed){}
    private final DriveTrain driveTrain;

    // constructor
    public TurnRight (final DriveTrain subsystem) {
        driveTrain = subsystem;
    }

  @Override
  public void execute() {
    double jantLeft = 0.75; 
    double jantRight = -0.7;
    boolean doneTurning = false;
    
    while (!doneTurning) { // ! is the same as "not" in English
        // boolean doneRu\\Turing
        //SmartDashboard
        driveTrain.tankDrive(jantLeft, jantRight) ;  
    }
  }
}
