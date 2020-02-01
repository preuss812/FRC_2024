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
        public static final int[] kLeftMotors   = {30, 31};
        public static final int[] kRightMotors  = {33, 36};
        public static final int kWinchMotor     = 32;
        public static final int kHookMotor      = 37;
        public static final int kIntakeMotorLeft = 38;
        public static final int kIntakeMotorRight = 39;

        public static final int kPDP = 1;
        public static final int kPCM = 40;
        public static final int kElevatorMotor = 35;

    }
    public static final class PCMConstants {
        public static final int[] kLiftPistons  = {0, 1};
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
        public static final double[] kBlueTargetRBG = {0.20, 0.46, 0.30};
        public static final double[] kGreenTargetRGB = {0.23, 0.57, 0.19};
        public static final double[] kRedTargetRGB = {0.41, 0.40, 0.19};
        public static final double[] kYellowTargetRGB = {0.39, 0.44, 0.16};
        public static final double kColorConfidenceThreshhold = 0.80;
        public static final int kColorProximityThreshhold = 300; // higher is closer, lower is further away
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
    }
}