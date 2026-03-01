package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.motorState;
import frc.robot.subsystems.ShooterOutSubsystem;

public class ShooterOutCommand extends Command{
    
    private final ShooterOutSubsystem m_subsystem;
    private BooleanSupplier rBumper;
    private BooleanSupplier lBumper;
    public ShooterOutCommand(ShooterOutSubsystem subsystem, BooleanSupplier rBumper, BooleanSupplier lBumber) {
        m_subsystem = subsystem;
        this.rBumper = rBumper;
        this.lBumper = lBumber;

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
        if (!(rBumper.getAsBoolean() && lBumper.getAsBoolean())) {
            if(rBumper.getAsBoolean()) {
                m_subsystem.setmotorTargetVelocity(ShooterConstants.kOutForwardLow + 300);
            }
            if (lBumper.getAsBoolean()) {
                m_subsystem.setmotorTargetVelocity(ShooterConstants.kOutForwardLow - 300);
            }
    
        }
        
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
