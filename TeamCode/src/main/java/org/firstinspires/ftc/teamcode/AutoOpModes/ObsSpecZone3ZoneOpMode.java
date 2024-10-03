package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.ObsSpecHi3Zone_commands;
import org.firstinspires.ftc.teamcode.CommandGroups.ObsSpecZone3Zone_commands;
import org.firstinspires.ftc.teamcode.Lib.g;

@Autonomous(name = "Obs Spec Zone 3 in zone", group = "Obs Zone")
public class ObsSpecZone3ZoneOpMode extends AutoCommandOpMode{

    public void initialize(){
        super.initialize();
        g.ROBOT.AutoCommands = new ObsSpecZone3Zone_commands();
    }

}

