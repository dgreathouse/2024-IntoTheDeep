package org.firstinspires.ftc.teamcode.Commands.Drive;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Lib.g;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * Drive the robot at a speed, angle for a certain time. Also rotate the robot to an angle while driving
 * Since the robot drives at a velocity the only changing variable to get a consistent distance is
 * the battery voltage which changes the time it takes to go from stop to full velocity. We hope this change
 * will not affect the accuracy to much. There is no PID on distance and the coast time will need
 * to be considered.
 */
public class AutoDriveTimeVel extends CommandBase {
    CommandOpMode m_opMode;
    DriveSubsystem m_drive;
    double m_driveAngle;
    double m_robotAngle;
    double m_timeOut_sec;
    double m_speed;
    double m_currentSpeed;

    double m_rotSpeed = g.DRIVE.ROTATE_MAX_SPEED;
    double m_rampUpTime_sec = 1.0; //Seconds
    double m_rampDownTime_sec = 0;

    PIDController rotPID = new PIDController(0.05, 0.051, 0);
    Timing.Timer m_timer;

    /**
     *
     * @param _driveAngle The drive angle in degrees CCW(-) CW(+)
     * @param _speed Speed from 0-1
     * @param _robotAngle The robot angle in degrees CCW(+) CW(-)
     * @param _timeOut_sec  The time to drive
     * @param _rampUpTime_sec Drive speed ramp up time in seconds
     * @param _rampDownTime_sec Drive speed ramp down time in seconds
     */
    public AutoDriveTimeVel(double _driveAngle, double _speed, double _robotAngle, double _timeOut_sec, double _rampUpTime_sec, double _rampDownTime_sec) {
        m_opMode = g.ROBOT.OpMode;
        m_drive = g.ROBOT.Drive;
        m_driveAngle = _driveAngle;
        m_speed = _speed;

        m_robotAngle = _robotAngle;
        m_timeOut_sec = _timeOut_sec;
        m_currentSpeed = m_speed;
        m_rampDownTime_sec = _rampDownTime_sec;
        m_rampUpTime_sec = _rampUpTime_sec;
    }
    /** Drive for the specified time. Drive ramp times are precalculated to a max of 0.75
     *

     * @param _driveAngle The drive angle in degrees CCW(-) CW(+)
     * @param _speed Speed from 0-1
     * @param _robotAngle The robot angle in degrees CCW(+) CW(-)
     * @param _timeOut_sec  The time to drive
     */
    public AutoDriveTimeVel( double _driveAngle, double _speed, double _robotAngle, double _timeOut_sec) {
        m_opMode = g.ROBOT.OpMode;
        m_drive = g.ROBOT.Drive;
        m_driveAngle = _driveAngle;
        m_speed = _speed;
        m_robotAngle = _robotAngle;
        g.DRIVE.RobotTargetAngle_deg = new Rotation2d(_robotAngle);
        m_timeOut_sec = _timeOut_sec;
        if (m_timeOut_sec >= 2.0) {
            m_rampUpTime_sec = 0.75;
            m_rampDownTime_sec = 0.75;
        } else {
            m_rampUpTime_sec = m_timeOut_sec * 0.50;
            m_rampUpTime_sec = MathUtils.clamp(m_rampUpTime_sec, 0, 0.75);
            m_rampDownTime_sec = m_timeOut_sec * 0.50;
            m_rampDownTime_sec = MathUtils.clamp(m_rampDownTime_sec, 0, 0.75);
        }
    }

    @Override
    public void initialize() {
        rotPID.reset();
        rotPID.setTolerance(0.05);
        m_drive.setMotorRunMode(Motor.RunMode.VelocityControl);
        rotPID.setIntegrationBounds(-0.2, 0.2);
        m_timer = new Timing.Timer((long) (m_timeOut_sec * 1000.0), TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute() {
     //   double rot = -rotPID.calculate(g.DRIVE.RobotAngle_deg, m_robotAngle);
        double currentTime_sec = m_timer.elapsedTime() / 1000.0;
        m_currentSpeed = m_drive.getRampSpeed(m_speed, m_timeOut_sec, currentTime_sec,m_rampUpTime_sec,m_rampDownTime_sec);


     //   rot = MathUtils.clamp(rot, -m_rotSpeed, m_rotSpeed);
        m_drive.drivePolar(m_driveAngle, m_currentSpeed,g.DRIVE.RobotTargetAngle_deg.getDegrees());
        m_opMode.telemetry.addData("CurrentSpeed", m_currentSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.done()) {
            m_drive.disableMotors();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean _interrupted) {
        m_drive.disableMotors();
    }
}

