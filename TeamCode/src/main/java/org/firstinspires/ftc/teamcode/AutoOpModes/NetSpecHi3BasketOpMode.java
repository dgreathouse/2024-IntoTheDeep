package org.firstinspires.ftc.teamcode.AutoOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroups.Net4SampleHi_commands;
import org.firstinspires.ftc.teamcode.CommandGroups.NetSpecHi3Basket_commands;
import org.firstinspires.ftc.teamcode.Lib.g;

@Autonomous(name = "Net Spec Hi 3 Hi Basket", group = "Net Zone")
public class NetSpecHi3BasketOpMode extends AutoCommandOpMode{

    public void initialize(){
        super.initialize();
        g.ROBOT.AutoCommands = new NetSpecHi3Basket_commands();
    }

}
