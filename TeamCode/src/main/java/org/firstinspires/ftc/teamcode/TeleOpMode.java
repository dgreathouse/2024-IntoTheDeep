package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Drive.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.DriveMode;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.g;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOp 1", group="Linear Opmode")
public class TeleOpMode extends CommandOpMode {
    private Hw hw;
    private DriveSubsystem m_drive;
    @Override
    public void initialize() {

        hw = new Hw(this);
        hw.init();

        // Create subsystems
        m_drive = new DriveSubsystem(this,hw);
        
        // Create Default Commands
        DriveDefaultCommand driveDefaultCommand = new DriveDefaultCommand(this, m_drive);
        // Set Default Commands
        m_drive.setDefaultCommand(driveDefaultCommand);
        // Set up buttons Driver
        // A=cross, B=circle, Y=triangle, X=square, back=share, start=option, guide=ps, (XBOX=PS4)
        //     Triangle
        // Square      Circle
        //     Cross
        // Set DriverModes
        Hw.gpDriver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.ANGLE_FIELD_CENTRIC), m_drive));
        Hw.gpDriver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.FIELD_CENTRIC), m_drive));
        Hw.gpDriver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> m_drive.setDriveMode(DriveMode.ROBOT_CENTRIC), m_drive));
        Hw.gpDriver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(new InstantCommand(() -> m_drive.setSpeedScale(0.5), m_drive),new InstantCommand(() -> m_drive.setSpeedScale(1.0), m_drive));

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
