package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class g {

    public static class ROBOT{
        public static CommandOpMode OpMode;
        public static DriveSubsystem Drive;

        public static GamepadEx GpDriver;
        public static GamepadEx GpOperator;
        public static IMU ControlHubImu;
        public static SequentialCommandGroup AutoCommands;
        public static void init(CommandOpMode _opMode){
            OpMode = _opMode;
            GpDriver = new GamepadEx(OpMode.gamepad1);
            GpOperator = new GamepadEx(OpMode.gamepad2);
            ControlHubImu = OpMode.hardwareMap.get(BHI260IMU.class,"imu");
        }

    }
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

        public static final PIDController ROT_PID = new PIDController(ROTATE_P, ROTATE_I, 0.0);


    }
    public static final class CAMERA {
        public static int TagOfInterest = 7;
        public static double AprilTagBearing = 0;
        public static double AprilTagRange = 0;
        public static double AprilTag_X = 0.01;
        public static double AprilTag_Y = 0.01;
        public static double AprilTag_Z = 0.01;


        public static final double FEET_PER_METER = 3.28084;
        //        public static final double fx = 1420.7594;
//        public static final double fy = 1420.9965;
//        public static final double cx = 630.8343;
//        public static final double cy = 381.3086;
        public static final double FX = 1121.6864;
        public static final double FY = 1133.7887;
        public static final double CX = 635.2146;
        public static final double CY = 465.3686;
        //public static final double tagsize = 0.0508;  // 2 inch
        public static final double TAG_SIZE = 0.127;  // 5 inch
        //public static final double tagsize = 0.1524;  // 6 inch
        public static  int numFramesWithoutDetection = 0;

        public static final float DECIMATION_HIGH = 3;
        public static final float DECIMATION_LOW = 2;
        public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        public static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


    }

}
