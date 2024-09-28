package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Lib.DriveMode;
import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.Kiwi3Drive;
import org.firstinspires.ftc.teamcode.Lib.TargetAngle;
import org.firstinspires.ftc.teamcode.Lib.g;

public class DriveSubsystem extends SubsystemBase {
    private final Hw m_hw;
    private final CommandOpMode m_opMode;
    private final Kiwi3Drive m_drive;  // TODO: Change to your drive class
    public PIDController m_rotPID;
    public DriveSubsystem(CommandOpMode _opMode, Hw _hw) {
        m_opMode = _opMode;
        m_hw = _hw;
        m_drive = new Kiwi3Drive(m_opMode, m_hw); // TODO: Change to your drive class
        m_drive.initHardware();
        m_rotPID = new PIDController(g.DRIVE.ROTATE_P,g.DRIVE.ROTATE_I,0);

        
    }
    /**
     *
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

    /** Used during autonomous */
    public void drivePolarFieldCentric(double _driveAngle, double _speed, double _robotHeading){
        double m_z = -m_rotPID.calculate(g.DRIVE.RobotAngle_deg, _robotHeading);
        m_drive.drivePolarAngleFieldCentric(_driveAngle, _speed, m_z);
    }
    public void setSpeedScale(double _scale){
        g.DRIVE.SpeedScale = _scale;
    }
    private void setRobotAngle(){
        g.DRIVE.RobotAngle_deg = Hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);    // TODO: Change to your orientation the ControlHub is mounted at to get the Robot Yaw
    }
    public void setRobotDesiredAngle(double _ang_deg){
        TargetAngle.setTargetAngle(_ang_deg);
    }
    public void setDriveMode(DriveMode _mode){
        g.DRIVE.Mode = _mode;
    }

    @Override
    public void periodic(){
        setRobotAngle();

    }
}
