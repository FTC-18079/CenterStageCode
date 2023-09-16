package org.firstinspires.ftc.teamcode.Chassis;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Chassis extends SubsystemBase {
    private MecanumDrive drive;
    private final Encoder m_lf, m_rf, m_lb, m_rb;

    private final double WHEEL_DIAMETER;

    public Chassis(MotorEx lf, MotorEx rf, MotorEx lb, MotorEx rb, double diameter){
        m_lf = lf.encoder;
        m_rf = rf.encoder;
        m_lb = lb.encoder;
        m_rb = rb.encoder;

        WHEEL_DIAMETER = diameter;

        drive = new MecanumDrive(lf, rf, lb, rb);
    }

    public void drive(double str, double fwd, double rot, double gyr, boolean isRobotOriented){
        if (isRobotOriented){
            drive.driveRobotCentric(str, fwd, rot);
        } else {
            drive.driveFieldCentric(str, fwd, rot, gyr);
        }
    }
}
