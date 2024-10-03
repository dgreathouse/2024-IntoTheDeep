package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.NetSpecHi3Basket_commands;
import org.firstinspires.ftc.teamcode.CommandGroups.ObsSpecHi3Zone_commands;
import org.firstinspires.ftc.teamcode.Lib.g;

@Autonomous(name = "Obs Spec Hi 3 in zone", group = "Obs Zone")
public class ObsSpecHi3ZoneOpMode extends AutoCommandOpMode{

    public void initialize(){
        super.initialize();
        g.ROBOT.AutoCommands = new ObsSpecHi3Zone_commands();
    }

}

