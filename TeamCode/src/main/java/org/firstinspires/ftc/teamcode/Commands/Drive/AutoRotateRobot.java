package org.firstinspires.ftc.teamcode.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.g;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoRotateRobot extends CommandBase {
    double m_angle;
    double m_timeOut_sec;
    double m_speed;
    DriveSubsystem m_drive;
    PIDController rotPID = new PIDController(0.015, 0.0075, 0);
    Timing.Timer m_timer;
    public AutoRotateRobot(double _angle, double _speed, double _timeOut_sec) {

        m_angle = _angle;
        m_speed = _speed;
        m_timeOut_sec = _timeOut_sec;
    }
    @Override
    public void initialize(){
        m_drive = g.ROBOT.Drive;
        rotPID.reset();
        rotPID.setTolerance(0.5);
        m_timer = new Timing.Timer((long)(m_timeOut_sec*1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
    }
    @Override
    public void execute(){
        double rot = -rotPID.calculate(g.DRIVE.RobotAngle_deg, m_angle);
        rot = MathUtils.clamp(rot,-m_speed,m_speed);
        m_drive.driveFieldCentric(0, 0, rot);
    }
    @Override
    public boolean isFinished(){
        if(m_timer.done() || rotPID.atSetPoint()){
            m_drive.disableMotors();
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean _interrupted){
        m_drive.disableMotors();
    }
}
