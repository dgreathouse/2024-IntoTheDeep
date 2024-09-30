package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.Net4SampleHi_commands;
import org.firstinspires.ftc.teamcode.Lib.g;

@Autonomous(name = "Net Zone 4 Hi Basket", group = "Net Zone")
public class Net4SampleHiOpMode extends AutoCommandOpMode{
    public Net4SampleHiOpMode() {
        g.ROBOT.AutoCommands = new Net4SampleHi_commands();
    }
}
