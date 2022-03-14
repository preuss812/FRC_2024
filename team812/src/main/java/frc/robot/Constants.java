/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANConstants {
        public static final int[] kLeftMotors   = {44, 7};
        public static final int[] kRightMotors  = {30, 6};
        public static final int kWinchMotor     = 80; // dko remove for 2022
        public static final int kHookMotor      = 81; // dko remove for 2022
        public static final int kIntakeMotorLeft = 82; // dko remove for 2022
        public static final int kIntakeMotorRight = 83; // dko remove for 2022
        public static final int kIntakeMotor = 39; // dko 20220221 Mr. Rupert went to single motor
        public static final int kSpinMotor = 84; // dko remove for 2022

        public static final int kPDP = 42;
        public static final int kPCM = 40;
        public static final int kElevatorMotor = 85; // dko remove 2022
        public static final int kElevatorMotorLeft = 32;
        public static final int kElevatorMotorRight = 33;
        public static final int kArmMotor = 31;
    }
    public static final class PCMConstants {
        public static final int[] kLiftPistons  = {0, 1}; // dko remove for 2022
        public static final int[] kGearShift = {2, 3};  // dko remove for 2022
        public static final int[] kBarHooks = {4,5}; // dko 20220221 need to physically verify
        public static final int[] kArmExtension = {6,7}; // dko 20220221 need to physicall verify
        public static final int kMinPresssure = 50; // minimum operating pressure for Arm control
    }
    public static final class OIConstants {
        public static final int kLeftJoystick = 0;
        public static final int kRightJoystick = 1;
        public static final int kControlBox = 2;
        public static final int kXboxController = 3;
        

        // Controlbox interfaces
        public static final int kControlBoxPotX = 0;
        public static final int kControlBoxPotY = 1;
        public static final int[] kControlBox3WaySwitch = {1,2}; // 3 position switch, see truth table below
        public static final int kControlBoxTottleButton = 3; // normally closed (1), pressed is open (0)
        public static final int kControlBoxSw1 = 4; // up is (0), down is (1) for all two position switches
        public static final int kControlBoxSw2 = 5;
        public static final int kControlBoxSw3 = 6;
        public static final int kControlBoxSw4 = 7;

        /* 
        kControlBox3WaySwitch implements the following states
           sw1   |   sw2   | 3-way position
        ---------+---------+---------------------
            0    |    0    | Left
            0    |    1    | Center
            1    |    0    | No such position
            1    |    1    | Right
        */

        // Xbox joystick constants
        public static final int kXboxAButton = 1;
        public static final int kXboxBButton = 2;
        public static final int kXboxXButton = 3;
        public static final int kXboxYButton = 4;
        public static final int kXboxLBumper = 5;
        public static final int kXboxRBumper = 6;
        public static final int kXboxSelect = 7;
        public static final int kXboxStart = 8;
        public static final int kXboxLTrigger = 9;
        public static final int kXboxRTrigger = 10;
    }
    public static final class AnalogIOConstants {
        public static final int kPressureTransducer = 0;
        public static final int kPressureOffset = -20;
        public static final int kPressureRange = 200;
    }

    public static final class EncoderConstants {
        public static final int kRightDriveEncoder = 0; // placeholder
        public static final int kLeftDriveEncoder = 1; // placeholder
        public static final int kElevatorEncoder = 2; // placeholder

        public static final double wheelDiameter = 7.25;
        public static final double ticksPerRevolution = 256;

        public static final double kEncoderDistanceFactor = (wheelDiameter * Math.PI) / ticksPerRevolution;
        //0.02832031, original factor
    }
    public static final class PidConstants {
        public static final double kProportionalDriveStraight = 0.05;
        public static final double kArm_kP = 3.0;
        public static final double kArm_kI = 0.0;
        public static final double kArm_kD = 0.0;
        public static final double kArm_kF = 0.0;
        public static final double kArm_rampRate = 0.5;
    }

    public static final class BallHandlingConstants {
        public static final double kBallIntakeSpeed = 0.7;
        public static final double kBallOutputSpeed = 1.0;
    }

    public static final class DriveTrainConstants {
        public static final double kOpenLoopRampRate = 0.75;
	
	    public static final double kHighSpeed = 1.0;
	    public static final double kTurnHighSpeed = 1.0;

	    public static final double kLowSpeed = 0.2;
	    public static final double kTurnLowSpeed = 0.8;

	    public static final double kLowLowSpeed = 0.08;
	    public static final double kTurnLowLowSpeed = 0.7;
    }

    public static final class ArmConstants {
        public static final double kArmScorePosition = 2100;
        public static final double kArmTopPositon = 4020; //3550;
        public static final double kArmHangPosition = 2550;
        public static final double kArmEndGamePosition = 2100;
        public static final double kArmBallGathering = 200;
        public static final double kArmThreshold = 20;
    }

    public static final int kLightRelay = 0;
    public static final int kTopLightRelay = 1;
}
