package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.g;

/**
 * The shoulder subsystem consists of one motor and encoder that manage the rotation of the arm.
 *
 */
public class ShoulderSubsystem extends SubsystemBase {
    MotorEx m_motor = new MotorEx(g.ROBOT.OpMode.hardwareMap, Hw.ShoulderMotor, Motor.GoBILDA.RPM_312);
    PIDController m_rotPID = new PIDController(g.SHOULDER.ROTATE_P, g.SHOULDER.ROTATE_I, 0.0);
    @Override
    public void periodic(){
        g.SHOULDER.angle_deg = getAngle();
    }

    /**
     * Rotate the shoulder to the desired angle. The starting angle is always zero.
     * Limit the should movement to known degrees. Use a PID controller to rotate to the desired angle.
     * @param _angle The angle in degrees to rotate the shoulder.
     */
    public void rotate(double _angle){

    }

    /**
     *
     * @return The angle in degrees the arm is currently at.
     */
    public double getAngle(){
        return 0;
    }

}
