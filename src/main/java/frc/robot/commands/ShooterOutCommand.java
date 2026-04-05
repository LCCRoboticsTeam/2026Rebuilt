package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.motorState;
import frc.robot.subsystems.ShooterOutSubsystem;

public class ShooterOutCommand extends Command{
    
    private final ShooterOutSubsystem m_subsystem;

    public ShooterOutCommand(ShooterOutSubsystem subsystem) {
        m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    public void initialize() {
        motorState m_motorState = m_subsystem.getMotorState();
        if (m_motorState == motorState.RUNNING) {
            m_subsystem.setmotorTargetVelocity(0);
            m_subsystem.setMotorState(motorState.STOPPED);
        } else {
            m_subsystem.setmotorTargetVelocity(ShooterConstants.kOutForwardLow);
            m_subsystem.setMotorState(motorState.RUNNING);
        }
    }

    public void execute() {
        
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
