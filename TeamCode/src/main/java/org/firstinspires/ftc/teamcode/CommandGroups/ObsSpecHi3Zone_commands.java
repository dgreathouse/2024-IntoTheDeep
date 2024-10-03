package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.g;

public class ObsSpecHi3Zone_commands extends SequentialCommandGroup {
/* Start on Observation side, Score pre installed specimen on high bar, then get the other
   3 samples and score in observation zone, park in observation zone.
* */
    public ObsSpecHi3Zone_commands(){
        addCommands(

                new AutoDriveTimeVel(45, 0.5, 45, 1),
                new AutoRotateRobot(0, 0.5, 2),

                new InstantCommand(g.ROBOT.OpMode::requestOpModeStop)
        );
    }

}
