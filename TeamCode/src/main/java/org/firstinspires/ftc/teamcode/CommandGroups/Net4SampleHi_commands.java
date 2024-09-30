package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Drive.AutoDriveTimeVel;
import org.firstinspires.ftc.teamcode.Commands.Drive.AutoRotateRobot;
import org.firstinspires.ftc.teamcode.Lib.g;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class Net4SampleHi_commands extends SequentialCommandGroup {

    public Net4SampleHi_commands(){
        addCommands(
                /* Drive to Basket*/                    new AutoDriveTimeVel(0, 0.7, 0, 1),

                // Rotate Arm to angle for basket
                // Extend Claw
                // Spin Claw to deposit sample
                // Stop spinning Claw
                // Retract Claw
                /* Rotate Robot to Sample 1 */          new AutoRotateRobot(5,.5, 2),
                // Rotate Arm to floor
                // Extend Arm
                // Spin Claw to intake the sample
                // Stop spinning Claw
                // Retract Claw
                // Rotate Arm to Basket angle
                // Extend Claw to Basket

                // Spin Claw to deposit sample
                // Stop spinning Claw
                // Retract Claw
                // Rotate Robot to Sample 2
                // Rotate Arm to floor
                // Extend Arm
                // Spin Claw to intake the sample
                // Stop spinning Claw
                // Retract Claw
                // Rotate Arm to Basket angle
                // Extend Claw to Basket

                // Spin Claw to deposit sample
                // Stop spinning Claw
                // Retract Claw
                // Rotate Robot to Sample 3
                // Rotate Arm to floor
                // Extend Arm
                // Spin Claw to intake the sample
                // Stop spinning Claw
                // Retract Claw
                // Rotate Arm to Basket angle
                // Extend Claw to Basket

                // Spin Claw to deposit sample
                // Stop spinning Claw
                // Retract Claw
                // Rotate Robot to Sample 4
                // Rotate Arm to floor
                // Extend Arm
                // Spin Claw to intake the sample
                // Stop spinning Claw
                // Retract Claw
                // Rotate Arm to Basket angle
                // Extend Claw to Basket

                // Spin Claw to deposit sample
                // Stop spinning Claw
                // Retract Claw
                // Drive to Observation zone to park

                new AutoDriveTimeVel(0, 0.7, 0, 1),
                new AutoDriveTimeVel(180, 0.7, 0, 1),
                new InstantCommand(g.ROBOT.OpMode::requestOpModeStop)
        );
    }

}
