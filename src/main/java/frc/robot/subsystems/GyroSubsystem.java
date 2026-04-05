package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase{
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    double gyro_last_read = 0.0;
    double gyro_current_read = 0.0;
    boolean gyro_working = false;

    public void periodic() {
        gyro_current_read = driveSubsystem.getHeading();
        if (gyro_last_read == gyro_current_read) {
            gyro_working = false;
        } else {
            gyro_working = true;
            gyro_last_read = gyro_current_read;
        }
        SmartDashboard.putBoolean("Gyro Working", gyro_working);
    }

}
