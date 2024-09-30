package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.g;

public class ArmSubsystem extends SubsystemBase {
    MotorEx m_leftMotor = new MotorEx(g.ROBOT.OpMode.hardwareMap, Hw.ArmExtensionLeftMotor, Motor.GoBILDA.RPM_1620);
    MotorEx m_rightMotor = new MotorEx(g.ROBOT.OpMode.hardwareMap, Hw.ArmExtensionRightMotor, Motor.GoBILDA.RPM_1620);
    PIDController m_rotPID = new PIDController(g.ARM.ROTATE_P, g.ARM.ROTATE_I, 0.0);
    @Override
    public void periodic(){
        g.ARM.position_mm = getPosition();
    }

    /**
     * Move the arm in and out to the set mm. Drive both left and right motors.
     * Limit the movement to known physical limits. Use a PID loop if needed to get to the correct
     * position.
     * @param _mm
     */
    public void move(double _mm){

    }

    /**
     *
     * @return Position of the arm in mm.
     */
    public double getPosition(){
        return 0;
    }
}
