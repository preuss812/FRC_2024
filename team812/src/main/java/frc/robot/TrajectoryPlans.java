// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

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
    public static FieldStep transformFieldStep(FieldStep step)
    {
        FieldStep result = step;
        switch (step) {
            case Done:
                result = FieldStep.Done;
                break;
            case Up:
                result = FieldStep.Up;
                break;
            case UpLeft:
                result = FieldStep.UpRight;
                break;
            case UpRight:
            result = FieldStep.UpLeft;
                break;
            case Down:
            result = FieldStep.Down;
                break;
            case DownLeft:
            result = FieldStep.DownRight;
                break;
            case DownRight:
            result = FieldStep.DownLeft;
                break;
            case Left:
            result = FieldStep.Right;
                break;
            case Right:
            result = FieldStep.Left;
                break;
            default:
                result = step;
        }
        return result;
    }
    public static class FieldSquare {
        public Translation2d center;
        public FieldStep move;
        FieldSquare(Translation2d center, FieldStep move) {
            this.center = center;
            this.move = move;
        }
    }
    public static class TrajectoryPlan {
        public FieldSquare[][] plan;
        public TrajectoryPlan(FieldSquare[][] plan) {
            this.plan = plan;
        }
    }

    public static TrajectoryPlan transformPlan(TrajectoryPlan trajectoryPlan, Transform2d transform) {
        TrajectoryPlan newTrajectoryPlan;
        FieldSquare[][] newPlan = new FieldSquare[8][4];
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 4; j++) {
                Translation2d newCenter = new Translation2d(FieldConstants.xMax - trajectoryPlan.plan[i][j].center.getX(), trajectoryPlan.plan[i][j].center.getY());
                FieldStep newMove = transformFieldStep(trajectoryPlan.plan[i][j].move);
                //newPlan[7-i][j] = new TrajectoryPlans.FieldSquare( trajectoryPlan.plan[7-i][j].center, newMove);
                newPlan[7-i][j] = new TrajectoryPlans.FieldSquare(newCenter, newMove);
            }
        }
        newTrajectoryPlan = new TrajectoryPlan(newPlan);
        return newTrajectoryPlan;
    }
    public static double dx = FieldConstants.xMax/8.0;
    public static double dy = FieldConstants.yMax/4.0;
    public static final TrajectoryPlan BlueAmpPlan = new TrajectoryPlan( new FieldSquare[][]
        {
            { // Colunm 0:
                new FieldSquare(new Translation2d(dx*0+1.0, dy*0+2.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*0+1.5, dy*1+1.5), FieldStep.UpRight),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*3+0.75), FieldStep.Done)
            },
            { // Column 1:
                new FieldSquare(new Translation2d(dx*1+1.0, dy*0+2.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*1+1.5), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*1+0.5, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*1+0.0, dy*3+1.0), FieldStep.Done)
            },
            { // Column 2:
                new FieldSquare(new Translation2d(dx*2+0.0, dy*0+2.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*1+1.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*3+1.0), FieldStep.Left)
            },
            { // Column 3:
                new FieldSquare(new Translation2d(dx*3+1.0, dy*0+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*1+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*2+1.5), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*3+0.5, dy*3+0.5), FieldStep.Left)
            },
            { // Column 4:
                new FieldSquare(new Translation2d(dx*4+1.0, dy*0+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*4+0.0, dy*1+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*4+0.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*4+2.0, dy*3+1.0), FieldStep.Left)
            },
            { // Column 5:
                new FieldSquare(new Translation2d(dx*5+1.0, dy*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*3+1.0), FieldStep.Left),
            },
            { // Column 6:
                new FieldSquare(new Translation2d(dx*6+1.0, dy*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*1+1.0), FieldStep.DownLeft), // To avoid the pillars of the Stage
                new FieldSquare(new Translation2d(dx*6+1.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*3+1.0), FieldStep.Left)
            },
            { // Column 7:
                new FieldSquare(new Translation2d(dx*7+0.0, dy*0+1.0), FieldStep.Left),
                new FieldSquare(new Translation2d(dx*7+0.0, dy*1+1.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*7+0.0, dy*2+1.0), FieldStep.UpLeft),
                new FieldSquare(new Translation2d(dx*7+0.0, dy*3+1.0), FieldStep.Left)
            }        
    });
    public static final TrajectoryPlan BlueSourcePlan = new TrajectoryPlan( new FieldSquare[][]
        {
            
            /* This version is smoother but sometimes crashes the tragectorygenerator. */
            { // Colunm 0:
                new FieldSquare(new Translation2d(dx*0+2.0, dy*0+2.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*0+1.5, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 1:
                new FieldSquare(new Translation2d(dx*1+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*1+0.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*1+0.5, dy*2+1.0), FieldStep.UpRight),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 2:
                new FieldSquare(new Translation2d(dx*2+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 3:
                new FieldSquare(new Translation2d(dx*3+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*2+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+0.5, dy*3+0.0), FieldStep.Down)
            },
            { // Column 4:
                new FieldSquare(new Translation2d(dx*4+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*4+0.0, dy*1+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+0.5, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 5:
                new FieldSquare(new Translation2d(dx*5+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*5+2.0, dy*2+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*3+1.0), FieldStep.DownRight),
            },
            { // Column 6:
                new FieldSquare(new Translation2d(dx*6+2.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*6+2.0, dy*1+0.0), FieldStep.Down), 
                new FieldSquare(new Translation2d(dx*6+1.0, dy*2+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 7:
                new FieldSquare(new Translation2d(dx*7+1.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*1+1.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*7+0.5, dy*2+0.0), FieldStep.DownLeft),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*3+1.0), FieldStep.Down)
            }   
                 
            /*
            { // Colunm 0:
                new FieldSquare(new Translation2d(dx*0+2.0, dy*0+2.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*0+1.5, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*0+2.0, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*0+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 1:
                new FieldSquare(new Translation2d(dx*1+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*1+0.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*1+0.5, dy*2+1.0), FieldStep.UpRight),
                new FieldSquare(new Translation2d(dx*1+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 2:
                new FieldSquare(new Translation2d(dx*2+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*2+1.0), FieldStep.Up),
                new FieldSquare(new Translation2d(dx*2+1.0, dy*3+1.0), FieldStep.Right)
            },
            { // Column 3:
                new FieldSquare(new Translation2d(dx*3+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*2+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*3+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 4:
                new FieldSquare(new Translation2d(dx*4+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*4+1.0, dy*1+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+0.5, dy*2+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*4+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 5:
                new FieldSquare(new Translation2d(dx*5+1.0, dy*0+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*1+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*5+2.0, dy*2+1.0), FieldStep.Right),
                new FieldSquare(new Translation2d(dx*5+1.0, dy*3+1.0), FieldStep.DownRight),
            },
            { // Column 6:
                new FieldSquare(new Translation2d(dx*6+1.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*6+2.0, dy*1+0.0), FieldStep.Down), 
                new FieldSquare(new Translation2d(dx*6+1.0, dy*2+1.0), FieldStep.DownRight),
                new FieldSquare(new Translation2d(dx*6+1.0, dy*3+1.0), FieldStep.Down)
            },
            { // Column 7:
                new FieldSquare(new Translation2d(dx*7+1.0, dy*0+1.0), FieldStep.Done),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*1+1.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*7+0.5, dy*2+0.0), FieldStep.Down),
                new FieldSquare(new Translation2d(dx*7+1.0, dy*3+1.0), FieldStep.Down)
            }  
            */
                  
    });
    public static final TrajectoryPlan RedAmpPlan = transformPlan(BlueAmpPlan, FieldConstants.AllianceTransformation[FieldConstants.RedAlliance]);
    public static final TrajectoryPlan RedSourcePlan = transformPlan(BlueSourcePlan, FieldConstants.AllianceTransformation[FieldConstants.RedAlliance]);
    public TrajectoryPlans() {

    }
}
