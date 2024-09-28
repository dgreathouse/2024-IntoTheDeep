package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Lib.Hw;
import org.firstinspires.ftc.teamcode.Lib.Kiwi3Drive;
import org.firstinspires.ftc.teamcode.Lib.g;

public class AutoDriveSubsystem extends SubsystemBase {
    private final Hw m_hw;
    private final CommandOpMode m_opMode;
    private final Kiwi3Drive m_drive;
    public PIDController m_rotPID;
    public AutoDriveSubsystem(CommandOpMode _opMode, Hw _hw){
        m_hw = _hw;
        m_opMode = _opMode;
        m_drive = new Kiwi3Drive(_opMode, _hw);
        m_drive.initHardware();
        m_rotPID = new PIDController(g.DRIVE.ROTATE_P,g.DRIVE.ROTATE_I,0);
    }
}
