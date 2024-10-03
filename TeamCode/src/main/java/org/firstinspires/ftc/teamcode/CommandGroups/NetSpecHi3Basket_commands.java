package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.g;

public class NetSpecHi3Basket_commands extends SequentialCommandGroup {
/* Start on NET side, Score pre installed specimen on high bar, then get the other
   3 samples and score in hi basket, park by touching the low ascent bar.
* */
    public NetSpecHi3Basket_commands(){
        addCommands(

                new AutoDriveTimeVel(45, 0.5, 45, 1),
                new AutoRotateRobot(0, 0.5, 2),

                new InstantCommand(g.ROBOT.OpMode::requestOpModeStop)
        );
    }

}
