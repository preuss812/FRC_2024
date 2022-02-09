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
        public static final int[] kLeftMotors   = {36, 33};
        public static final int[] kRightMotors  = {30, 31};
        public static final int kWinchMotor     = 7; // dko 20200210 device 41 does not exist
        public static final int kHookMotor      = 42;
        public static final int kIntakeMotorLeft = 32;
        public static final int kIntakeMotorRight = 37;
        public static final int kSpinMotor = 6;

        public static final int kPDP = 1;
        public static final int kPCM = 40;
        public static final int kElevatorMotor = 35;
    }
    public static final class PCMConstants {
        public static final int[] kLiftPistons  = {0, 1};
        public static final int[] kGearShift = {2, 3};
    }
    public static final class OIConstants {
        public static final int kLeftJoystick = 1;
        public static final int kRightJoystick = 0;
        public static final int kXboxController = 2;
        public static final int kControlBox = 3;

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
    public static final class ColorConstants {
        //Color values at the Preuss School Lab
        public static final double[] kBlueTargetRBG   = {0.1213, 0.4416, 0.4372}; // {0.20, 0.46, 0.30};
        public static final double[] kGreenTargetRGB  = {0.1784, 0.6201, 0.2004}; //{0.23, 0.57, 0.19};
        public static final double[] kRedTargetRGB    = {0.5750, 0.3070, 0.1230}; //{0.41, 0.40, 0.19};
        public static final double[] kYellowTargetRGB = {0.3530, 0.5617, 0.0849}; //{0.39, 0.44, 0.16};
        /*
        // Color Values at Del Mar Fairgrounds 2020-03-06
        public static final double[] kBlueTargetRBG   = {0.1521, 0.4367, 0.4111};
        public static final double[] kYellowTargetRGB = {0.3630, 0.5183, 0.1201};
        public static final double[] kRedTargetRGB    = {0.5593, 0.3203, 0.1201};
        public static final double[] kGreenTargetRGB  = {0.1967, 0.5539, 0.2492};
        */

        public static final double kColorConfidenceThreshhold = 0.80;
        public static final int kColorProximityThreshhold = 300; // higher is closer, lower is further away
        public static final int kColorRed = 0;
        public static final int kColorYellow = 1;
        public static final int kColorBlue = 2;
        public static final int kColorGreen = 3;
        public static final int kColorUnknown = 4;

        public static final String[] kColorNames = {"red","yellow","blue","green","unknown"};
        public static final int kColorCorrection[][] = {
            // columns are for color to infer given the color detected
            {kColorRed,    kColorYellow, kColorUnknown,kColorYellow, kColorRed},     // Last saw red
            {kColorUnknown,kColorYellow, kColorBlue,   kColorYellow, kColorYellow},  // Last saw yellow
            {kColorUnknown,kColorGreen,  kColorBlue,   kColorGreen,  kColorBlue},    // last saw blue
            {kColorRed,    kColorGreen,  kColorUnknown,kColorGreen,  kColorGreen},   // last saw green
            {kColorRed,    kColorUnknown,kColorBlue,   kColorUnknown,kColorUnknown}  // last saw unknown
       };
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

    public static final class SpinConstants {
        public static final double kSpinMotorSpeed = 1.0;
        public static final int kColorRotationCountMax = 7;
        public static final double kSpinTimeout = 12.0;
    }
    
    public static final class PositionWheelConstants {
        public static final double kPositionWheelMotorSpeed = 0.5;
        public static final int kExpectedSamplesPerSlice = 10;
        public static final double kPositionWheelTimeout = 8.0;
    }

    public static final class PidConstants {
        public static final double kProportionalDriveStraight = 0.05;
    }

    public static final class BallHandlingConstants {
        public static final double kBallIntakeSpeed = 0.7;
        public static final double kBallOutputSpeed = 1.0;
    }

    public static final class DriveTrainConstants {
        public static final double kOpenLoopRampRate = 0.75;
    }

    public static final int kLightRelay = 0;
}
