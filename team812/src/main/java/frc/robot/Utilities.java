/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


/**
 * Add your docs here.
 */
public class Utilities {
    
    private static Alliance m_alliance = null;
    private static boolean m_isBlueAlliance = false;
    private static boolean m_isRedAlliance = false;

    public static double scaleDouble(final double input, final double to_min, final double to_max) {
            final double from_min = -1.0;
            final double from_max = 1.0;
        double x;
        double scaled_x = 0.0;
        if( to_max > to_min  && from_max > from_min )
        {
            x =  input;
            scaled_x = ((x - from_min) * (to_max - to_min)) / 
                        (to_max - to_min) +
                        to_min;
        }
        return scaled_x;    
    }

    public static void setAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            m_isBlueAlliance = (alliance.get() == Alliance.Blue);  // Remember which alliance we are in.
            m_isRedAlliance =  (alliance.get() == Alliance.Red);
        }
        SmartDashboard.putBoolean("BlueAlliance", m_isBlueAlliance);   
    }
    
    public static boolean isRedAlliance() {
        return m_isRedAlliance;
    }

    public static boolean isBlueAlliance() {
        return m_isBlueAlliance;
    }

    public static void toSmartDashboard(String label, Pose2d pose) {
        SmartDashboard.putString(label, String.format("(%4.2f,%4.2f) %2.0f", pose.getX(), pose.getY(), pose.getRotation().getDegrees()) );
    }

    public static Pose2d Pose180(Pose2d pose) {
        Rotation2d rotate180 = new Rotation2d(Math.PI);
        Rotation2d newRotation = pose.getRotation().rotateBy(rotate180);
        return new Pose2d(pose.getX(), pose.getY(), newRotation);
    }

    public static Pose2d nearPose(Pose2d pose, double distance) {
        Rotation2d rotate180 = new Rotation2d(Math.PI);
        Translation2d rotatedDistance = new Translation2d(distance, 0).rotateBy(pose.getRotation());
        Rotation2d newRotation = pose.getRotation().rotateBy(rotate180);
        return new Pose2d(pose.getX() + rotatedDistance.getX(), pose.getY() + rotatedDistance.getY(), newRotation);
    }
}