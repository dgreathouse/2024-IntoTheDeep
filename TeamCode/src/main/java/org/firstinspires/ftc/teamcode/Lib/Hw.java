package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.IMU;

public class Hw {

    // Control Hub
    public static String DriveFrontLeft = "fl";
    public static String DriveFrontRight = "fr";
    public static String DriveBackLeft = "bl";
    public static String DriveBackRight = "br";
    public static String DriveBack = "b"; // For Kiwi 3 Wheel Drive only

    public static String ClawRotateServo = "cr";
    public static String ClawIntakeServo = "ci";

    // Expansion hub
    public static String ShoulderLeftMotor = "sl";
    public static String ShoulderRightMotor = "sr";

    public static String ArmExtensionLeftMotor = "el";
    public static String ArmExtensionRightMotor = "er";


}
