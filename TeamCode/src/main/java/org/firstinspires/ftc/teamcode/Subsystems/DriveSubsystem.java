package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.DriveMode;
import org.firstinspires.ftc.teamcode.Lib.Kiwi3Drive;
import org.firstinspires.ftc.teamcode.Lib.TargetAngle;
import org.firstinspires.ftc.teamcode.Lib.g;

public class DriveSubsystem extends SubsystemBase {

    private final CommandOpMode m_opMode;
    private final Kiwi3Drive m_drive;  // TODO: Change to your drive class
    public PIDController m_rotPID;

    public DriveSubsystem() {
        m_opMode = g.ROBOT.OpMode;

        m_drive = new Kiwi3Drive(); // TODO: Change to your drive class
        m_drive.initHardware();
        m_rotPID = g.DRIVE.ROT_PID;
    }
    /**
     * @param _strafe The forward speed in +/- 1 left is positive
     * @param _forward The strafe speed in +/- 1 forward is positive
     * @param _zRotation The rotation speed in +/- 1 CCW/left is positive
     */
    public void driveRobotCentric(double _strafe, double _forward, double _zRotation){
        m_drive.driveRobotCentric(_strafe,_forward,_zRotation);
    }
    public void driveFieldCentric(double _strafe, double _forward, double _zRotation){
        m_drive.driveFieldCentric(_strafe, _forward, _zRotation);
    }

    /**
     *
     * @param _driveAngle The angle the robot should drive
     * @param _speed The speed to drive (+/- 1)
     * @param _robotAngle The angle the robot should face while driving
     */
    public void drivePolar(double _driveAngle, double _speed, double _robotAngle){
        double m_z = -m_rotPID.calculate(g.DRIVE.RobotAngle_deg, _robotAngle);
        m_drive.drivePolarAngleFieldCentric(_driveAngle, _speed, m_z);
    }
    public double getRampSpeed(double _speed, double _timeOut, double _currentTime, double _rampUpTime, double _rampDownTime){
        double currentSpeed = 0;
        if (_currentTime < _timeOut && _currentTime > _timeOut - _rampDownTime) { // In the ramp down time
            currentSpeed = _speed * (_timeOut - _currentTime) / _rampDownTime;
        } else if (_currentTime < _rampUpTime) {// In the ramp up time
            currentSpeed = _speed * _currentTime / _rampUpTime;
        } else { // past the ramp up time and not in ramp down time
            currentSpeed = _speed;
        }
        return currentSpeed;
    }
    public void setSpeedScale(double _scale){
        g.DRIVE.SpeedScale = _scale;
    }
    private void setRobotAngle(){
        g.DRIVE.RobotAngle_deg = g.ROBOT.ControlHubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);    // TODO: Change to your orientation the ControlHub is mounted at to get the Robot Yaw
    }
    public void setRobotDesiredAngle(double _ang_deg){
        TargetAngle.setTargetAngle(_ang_deg);
    }
    public void setDriveMode(DriveMode _mode){
        g.DRIVE.Mode = _mode;
    }
    public void disableMotors(){
        m_drive.disableMotors();
    }

    public void setMotorRunMode(Motor.RunMode _mode){
        m_drive.setMotorRunMode(_mode);
    }
    @Override
    public void periodic(){
        setRobotAngle();
        m_opMode.telemetry.addData("Robot Angle", "%3.3f",g.DRIVE.RobotAngle_deg);
    }
}
