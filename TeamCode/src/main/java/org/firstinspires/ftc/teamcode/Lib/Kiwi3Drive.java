package org.firstinspires.ftc.teamcode.Lib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;


public class Kiwi3Drive extends RobotDrive {

    private CommandOpMode m_opMode;
    MotorEx m_left, m_right, m_back;
    double[] m_speeds = new double[3];
    double m_strafe, m_forward;
    Vector2d m_vector;
    private Vector2d m_lVector, m_rVector, m_bVector;


    public Kiwi3Drive() {
        m_opMode = g.ROBOT.OpMode;

        m_lVector = new Vector2d(Math.cos(Math.toRadians(30.0)), Math.sin(Math.toRadians(30.0)));
        m_rVector = new Vector2d(Math.cos(Math.toRadians(150.0)), Math.sin(Math.toRadians(150.0)));
        m_bVector = new Vector2d(Math.cos(Math.toRadians(270.0)),  Math.sin(Math.toRadians(270.0)));
    }
    public void initHardware(){
        m_left = new MotorEx(m_opMode.hardwareMap, Hw.DriveFrontLeft, Motor.GoBILDA.RPM_435);
        m_left.setInverted(true);
        m_left.setRunMode(Motor.RunMode.VelocityControl);
        m_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_left.setDistancePerPulse(g.DRIVE.INCH_PER_COUNT);
        m_left.encoder.setDirection(Motor.Direction.FORWARD);
        m_left.setVeloCoefficients(g.DRIVE.MOTOR_VEL_P,g.DRIVE.MOTOR_VEL_I,0);

        m_right = new MotorEx(m_opMode.hardwareMap, Hw.DriveFrontRight, Motor.GoBILDA.RPM_435);
        m_right.setInverted(true);
        m_right.setRunMode(Motor.RunMode.VelocityControl);
        m_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_right.setDistancePerPulse(g.DRIVE.INCH_PER_COUNT);
        m_right.encoder.setDirection(Motor.Direction.REVERSE);
        m_right.setVeloCoefficients(g.DRIVE.MOTOR_VEL_P,g.DRIVE.MOTOR_VEL_I,0);

        m_back = new MotorEx(m_opMode.hardwareMap, Hw.DriveBack, Motor.GoBILDA.RPM_435);
        m_back.setInverted(true);
        m_back.setRunMode(Motor.RunMode.VelocityControl);
        m_back.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_back.setDistancePerPulse(g.DRIVE.INCH_PER_COUNT);
        m_back.encoder.setDirection(Motor.Direction.REVERSE);
        m_back.setVeloCoefficients(g.DRIVE.MOTOR_VEL_P,g.DRIVE.MOTOR_VEL_I,0);
    }
    public void driveRobotCentric(double _strafe, double _forward, double _rotate){
        // Set the class values before clipping
        m_strafe = _strafe;
        m_forward = _forward;

        // Limit/Clip the values and scale if needed
        _strafe = clipRange(_strafe);
        _forward = clipRange(_forward);
        _rotate = clipRange(_rotate);

        m_vector =  new Vector2d(_forward,_strafe);

        driveMotors(m_vector, _rotate);

    }
    public void driveFieldCentric(double _strafe, double _forward, double _rotate){
        m_strafe = _strafe;
        m_forward = _forward;

        // Limit/Clip the values and scale if needed
        _strafe = clipRange(_strafe);
        _forward = clipRange(_forward);
        _rotate = clipRange(_rotate);

        m_vector =  new Vector2d(_forward,_strafe).rotateBy(g.DRIVE.RobotAngle_deg);

        driveMotors(m_vector, _rotate);

    }
    private void driveMotors(Vector2d _vector, double _rotate){


        m_speeds[0] = _vector.scalarProject(m_lVector);
        m_speeds[1] = _vector.scalarProject(m_rVector);
        m_speeds[2] = _vector.scalarProject(m_bVector);

        normalize(m_speeds, g.DRIVE.SpeedScale);

        m_left.set(m_speeds[0] + _rotate);
        m_right.set(m_speeds[1] + _rotate);
        m_back.set(m_speeds[2] + _rotate);
    }

    public void drivePolarAngleFieldCentric(double _driveAngle, double _speed, double _rotate){
        double x = Math.sin(Math.toRadians(_driveAngle)) * _speed;
        double y = Math.cos(Math.toRadians(_driveAngle)) * _speed;
        driveFieldCentric(x,y,_rotate);

    }
    public void disableMotors(){
        setMotorRunMode(Motor.RunMode.RawPower);

        m_left.set(0);
        m_right.set(0);
        m_back.set(0);
    }
    public void setMotorRunMode(Motor.RunMode _mode){
        m_left.setRunMode(_mode);
        m_right.setRunMode(_mode);
        m_back.setRunMode(_mode);
    }

    @Override
    public void stop() {

    }
}
