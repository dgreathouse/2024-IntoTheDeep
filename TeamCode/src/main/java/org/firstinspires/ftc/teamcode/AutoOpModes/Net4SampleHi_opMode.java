package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.Net4SampleHi_commands;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveDefaultCommand;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDriveSubsystem;

@Autonomous(name = "Net Zone 4 Hi Basket", group = "Net Zone")
public class Net4SampleHi_opMode extends CommandOpMode {
    Hw m_hw;
    AutoDriveSubsystem m_drive;
    AutoDriveDefaultCommand driveDefaultCommand;
    Net4SampleHi_commands m_commands;

    @Override
    public void initialize() {
        m_hw = new Hw(this);
        m_hw.init();

        m_drive = new AutoDriveSubsystem(this,m_hw);

        driveDefaultCommand = new AutoDriveDefaultCommand(this,m_drive);
        m_drive.setDefaultCommand(driveDefaultCommand);

        m_commands = new Net4SampleHi_commands(this, m_drive);

        //register(m_drive);

    }
    @Override
    public void runOpMode() throws InterruptedException{
        initialize();

        waitForStart();

        CommandScheduler.getInstance().schedule(m_commands);
        // run the scheduler
        while (!isStopRequested() || opModeIsActive()) {
            run();

            telemetry.update();

        }
        reset();

    }
}
