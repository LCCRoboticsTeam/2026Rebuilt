package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterInSubsystem;
import frc.robot.Constants.motorState;
import frc.robot.Constants.ShooterConstants;

public class ShooterInCommand extends Command{

    private final ShooterInSubsystem m_subsystem;
    
    public ShooterInCommand(ShooterInSubsystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    public void initialize() {
        motorState m_motorstate = m_subsystem.getMotorState();
        if (m_motorstate == motorState.RUNNING) {
            m_subsystem.setmotorTargetVelocity(0);
            m_subsystem.setMotorState(motorState.STOPPED);
        } else {
            m_subsystem.setmotorTargetVelocity(ShooterConstants.kInForward);
            m_subsystem.setMotorState(motorState.RUNNING);
        }
    }

    public void exexute() {

    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
    
}
