package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Drive.DriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.g;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class AutoCommandOpMode extends CommandOpMode {

    @Override
    public void initialize() {
        g.ROBOT.init(this);

        g.ROBOT.Drive = new DriveSubsystem();

    }

    @Override
    public void runOpMode() throws InterruptedException{

        initialize();

        waitForStart();

        CommandScheduler.getInstance().schedule(g.ROBOT.AutoCommands);
        // run the scheduler
        while (!isStopRequested() || opModeIsActive()) {
            run();

            telemetry.update();

        }
        reset();

    }
}
