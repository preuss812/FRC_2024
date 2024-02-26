// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/** 
 * This class supplies data to create trajectories from any point on the field to the AMP or SOURCE
 * The hope is that these will enable semi-automatic driving from to and from those key field positions.
 * The strategy is the divide the field into 2 meter squares and then create plans based on the
 * best path to travel from each starting square.
 */
public class TrajectoryPlans {
    public enum FieldStep {
        Done,
        Up,
        UpLeft,
        UpRight,
        Down,
        DownLeft,
        DownRight,
        Left,
        Right
    }

    public class FieldSquare {
        public Translation2d center;
        public FieldStep move;
        FieldSquare(Translation2d center, FieldStep move) {
            this.center = center;
            this.move = move;
        }
    }
    public class TrajectoryPlan {
        public FieldSquare[][] plan;
        public TrajectoryPlan(FieldSquare[][] plan) {
            this.plan = plan;
        }
    }

    public static FieldSquare[][]  BlueAmpPlan;
    
    public TrajectoryPlans() {

    BlueAmpPlan = new FieldSquare[][]
        {
            { // Colunm 0:
                new FieldSquare(new Translation2d(2.0*0+1.0, 2.0*0+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(2.0*0+1.0, 2.0*1+1.0), FieldStep.UpRight),
                new FieldSquare(new Translation2d(2.0*0+1.0, 2.0*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(2.0*0+1.0, 2.0*3+1.0), FieldStep.Done)
            },
            { // Column 1:
                new FieldSquare(new Translation2d(2.0*1+1.0, 2.0*0+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(2.0*1+1.0, 2.0*1+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*1+1.0, 2.0*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(2.0*1+1.0, 2.0*3+1.0), FieldStep.Done)
            },
            { // Column 2:
                new FieldSquare(new Translation2d(2.0*2+1.0, 2.0*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*2+1.0, 2.0*1+1.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(2.0*2+1.0, 2.0*2+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*2+1.0, 2.0*3+1.0), FieldStep.Left)
            },
            { // Column 3:
                new FieldSquare(new Translation2d(2.0*3+1.0, 2.0*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*3+1.0, 2.0*1+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(2.0*3+1.0, 2.0*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(2.0*3+1.0, 2.0*3+1.0), FieldStep.Left)
            },
            { // Column 4:
                new FieldSquare(new Translation2d(2.0*4+1.0, 2.0*0+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(2.0*4+1.0, 2.0*1+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(2.0*4+1.0, 2.0*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(2.0*4+1.0, 2.0*3+1.0), FieldStep.Left)
            },
            { // Column 5:
                new FieldSquare(new Translation2d(2.0*5+1.0, 2.0*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*5+1.0, 2.0*1+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(2.0*5+1.0, 2.0*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(2.0*5+1.0, 2.0*3+1.0), FieldStep.Left),
            },
            { // Column 6:
                new FieldSquare(new Translation2d(2.0*6+1.0, 2.0*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*6+1.0, 2.0*1+1.0), FieldStep.DownRight), // To avoid the pillars of the Stage
                new FieldSquare(new Translation2d(2.0*6+1.0, 2.0*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(2.0*6+1.0, 2.0*3+1.0), FieldStep.Left)
            },
            { // Column 7:
                new FieldSquare(new Translation2d(2.0*7+1.0, 2.0*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*7+1.0, 2.0*1+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*7+1.0, 2.0*2+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(2.0*7+1.0, 2.0*3+1.0), FieldStep.Left)
            }        
        };
    }
}