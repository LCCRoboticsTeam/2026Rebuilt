package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase{
    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kUSB1);
    double gyro_last_read = 0.0;
    double gyro_current_read = 0.0;
    boolean gyro_working = false;

    public void periodic() {
        gyro_current_read = m_gyro.getAngle();
        if (gyro_last_read == gyro_current_read) {
            gyro_working = false;
            SmartDashboard.putBoolean("Gyro Working", gyro_working);
        } else {
            gyro_working = true;
            gyro_last_read = gyro_current_read;
            SmartDashboard.putBoolean("Gyro Working", gyro_working);
        }
        //SmartDashboard.putBoolean("Gyro Working", gyro_working);
    }

}
