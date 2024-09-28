package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class g {

    public static class DRIVE {
        // Variables
        public static volatile double SpeedScale = 1.0;
        public static volatile double RobotAngle_deg = 0.0;
        public static volatile Rotation2d RobotTargetAngle_deg = new Rotation2d(0);
        public static volatile DriveMode Mode = DriveMode.ANGLE_FIELD_CENTRIC;

        // CONSTANTS
        public static final double CPR = Motor.GoBILDA.RPM_435.getCPR();
        public static final double WHEEL_DIAMETER_IN = 3.78;
        public static final double WHEEL_CIRCUMFERENCE_IN = Math.PI * WHEEL_DIAMETER_IN;
        public static final double INCH_PER_COUNT = WHEEL_CIRCUMFERENCE_IN / CPR;
        public static final double ACHIEVABLE_MAX_TICKS_PER_SECOND = 2781.1;

        public static final double MOTOR_VEL_P = 1.0;
        public static final double MOTOR_VEL_I = 0.0;

        public static final double ROTATE_P = 1.0;
        public static final double ROTATE_I = 0.0;
        public static final double ROTATE_FIELD_CENTRIC_SCALE = 0.4;
        public static final double TARGET_ANGLE_DEADBAND = 0.8;


    }


}
