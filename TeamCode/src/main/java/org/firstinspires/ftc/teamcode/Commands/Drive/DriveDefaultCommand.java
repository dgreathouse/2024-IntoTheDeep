package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.TargetAngle;
import org.firstinspires.ftc.teamcode.Lib.g;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class DriveDefaultCommand extends CommandBase {
    // Declare a variable called "drive" of type "DriveSubsystem"
    DriveSubsystem m_drive;
    // Declare a variable called "opMode" of type "CommandOpMode"
    CommandOpMode m_opMode;
    // Create local variables of type double to store the stick X,Y,Z values and Angle of robot.
    double m_x, m_y, m_z, m_lx, m_ly, m_hyp;
    PIDController rotPID;
    GamepadEx gpDriver;
    GamepadEx gpOperator;

    public DriveDefaultCommand() {

        m_drive = g.ROBOT.Drive;    // Set the local "m_drive" variable to the parameter "_drive"
        m_opMode = g.ROBOT.OpMode;  // Set the local "opMode" variable to the parameter "_opMode"
        gpDriver = g.ROBOT.GpDriver;
        gpOperator = g.ROBOT.GpOperator;
        addRequirements(m_drive);
    }
    @Override
    public void initialize(){
        rotPID = g.DRIVE.ROT_PID;
    }
    @Override
    public void execute(){
        // Read the Stick values
        m_y = -gpDriver.getRightY();
        m_x = gpDriver.getRightX();
        m_z = gpDriver.getLeftX();
        // Scale the turning rotation down
        if(Math.abs(m_z) > 0.2) {
            m_z = Math.signum(m_z) * (Math.abs(m_z) - 0.2);
            m_z = m_z * 0.35;
        }else {
            m_z = 0;
        }
        switch(g.DRIVE.Mode){
            case ROBOT_CENTRIC:
                m_drive.driveRobotCentric(m_x, m_y, m_z);
                break;
            case FIELD_CENTRIC:
                m_drive.driveFieldCentric(m_x,m_y,m_z);
                break;
            case ANGLE_FIELD_CENTRIC:
                m_lx = gpDriver.getLeftX();
                m_ly = gpDriver.getLeftY();
                m_hyp = Math.hypot(m_lx, m_ly);
                if(Math.abs(m_hyp) > g.DRIVE.TARGET_ANGLE_DEADBAND){
                    TargetAngle.setTargetAngle(Math.toDegrees(Math.atan2(m_lx,m_ly)));
                }
                m_z = -rotPID.calculate(g.DRIVE.RobotAngle_deg, g.DRIVE.RobotTargetAngle_deg.getDegrees());
                m_drive.driveFieldCentric(m_x,m_y,m_z);
                break;
            case ROTATE_FIELD_CENTRIC:
                m_z = g.DRIVE.ROTATE_FIELD_CENTRIC_SCALE;
                m_drive.driveFieldCentric(m_x,m_y,m_z);
                break;
            case TANK:
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + g.DRIVE.Mode);
        }
    }
    @Override
    public void end(boolean _interrupted){

    }
}
