package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.g;

public class IntakeSubsystem extends SubsystemBase {
    MotorEx m_motor = new MotorEx(g.ROBOT.OpMode.hardwareMap, Hw.IntakeMotor, Motor.GoBILDA.RPM_1620);

    @Override
    public void periodic(){
        g.INTAKE.speed = m_motor.getVelocity();
    }


    public void spin(double _speed){

    }


}
