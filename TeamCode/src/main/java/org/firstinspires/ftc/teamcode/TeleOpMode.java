package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandGroups.Net4SampleHi_commands;
import org.firstinspires.ftc.teamcode.Commands.Arm.ArmDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Drive.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.Commands.Shoulder.ShoulderDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.DriveMode;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.g;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShoulderSubsystem;

@TeleOp(name="TeleOp 1", group="Linear Opmode")
public class TeleOpMode extends CommandOpMode {

    @Override
    public void initialize() {

        g.ROBOT.init(this);

        // Create subsystems
        g.ROBOT.Drive = new DriveSubsystem();
//        g.ROBOT.Shoulder = new ShoulderSubsystem();
//        g.ROBOT.Arm = new ArmSubsystem();
//        g.ROBOT.Intake = new IntakeSubsystem();

        DriveSubsystem m_drive = g.ROBOT.Drive;
        m_drive.setMotorRunMode(Motor.RunMode.RawPower);
        
        // Create Default Commands
        DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand();
//        ShoulderDefaultCommand shoulderDefaultCommand = new ShoulderDefaultCommand();
//        ArmDefaultCommand armDefaultCommand = new ArmDefaultCommand();
//        IntakeDefaultCommand intakeDefaultCommand = new IntakeDefaultCommand();

        // Set Default Commands
        m_drive.setDefaultCommand(driveDefaultCommand);
//        g.ROBOT.Shoulder.setDefaultCommand(shoulderDefaultCommand);
//        g.ROBOT.Arm.setDefaultCommand(armDefaultCommand);
//        g.ROBOT.Intake.setDefaultCommand(intakeDefaultCommand);


        //          PS4(XBOX)
        //---------------------------------
        // Share(Back)        Option(Start)
        //                        Triangle(Y)
        //    DPAD         Square(X)      Circle(B)
        //                         Cross(A)
        //            PS(Guide)

        // Set up buttons Driver
        // Set DriverModes
        g.ROBOT.GpDriver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.ANGLE_FIELD_CENTRIC), m_drive));
        g.ROBOT.GpDriver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.FIELD_CENTRIC), m_drive));
        g.ROBOT.GpDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.ROBOT_CENTRIC), m_drive));
        // Toggle Drive Speed Scale
        g.ROBOT.GpDriver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(new InstantCommand(() -> m_drive.setSpeedScale(0.5), m_drive),new InstantCommand(() -> m_drive.setSpeedScale(1.0), m_drive));

        // Set up buttons for Operator


    }
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.update();

        }
        reset();

    }
}
