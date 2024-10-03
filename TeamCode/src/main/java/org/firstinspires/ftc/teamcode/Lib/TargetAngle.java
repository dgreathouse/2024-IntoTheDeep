package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.geometry.Rotation2d;

public class TargetAngle {
    public static void setTargetAngle(double _x, double _y){
        double angle = Math.toDegrees(Math.atan2(_x,_y));
        double setAngle = 0.0;
        if(angle >= -22.5 && angle <= 22.5){                                        // North
            setAngle = 0.0;
        }else if(angle >= -67.5 && angle <= -22.5){                                 // North east
            setAngle = -45.0;
        }else if(angle >= -112.5 && angle < -67.5){                                 // East
            setAngle = -90.0;
        }else if(angle >= -157.5 && angle < -112.5){                                // South East
            setAngle = -135.0;
        }else if((angle >= 157.5 && angle <= 180.0) || (angle <= -157.5 && angle > -179.99)){ // South
            setAngle = 180.0;
        }else if(angle <= 67.5 && angle > 22.5){                                    // North West
            setAngle = 45.0;
        }else if(angle <= 112.5 && angle > 67.5){                                   // West
            setAngle = 90.0;
        }else if(angle <= 157.5 && angle > 112.5){                                  // South West
            setAngle = 135.0;
        }else{
            setAngle = 0.0;
        }
        g.DRIVE.RobotTargetAngle_deg = Rotation2d.fromDegrees(setAngle);
    }

    public static void setTargetAngle(double _angDeg){
        g.DRIVE.RobotTargetAngle_deg = Rotation2d.fromDegrees(_angDeg);
    }
}
