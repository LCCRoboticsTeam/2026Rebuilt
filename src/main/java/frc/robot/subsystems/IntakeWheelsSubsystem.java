// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class IntakeWheelsSubsystem extends SubsystemBase {

    private final SparkMax m_intakeSpark;
    private final RelativeEncoder m_sparkEncoder;
    private final SparkClosedLoopController m_intakeClosedLoopController;

    private final SparkMaxConfig m_wheelMotorConfig;
    private double targetVelocity = 0.0;

  /** Creates a new ExampleSubsystem. */
  public IntakeWheelsSubsystem() {
    m_intakeSpark = new SparkMax(IntakeConstants.kShooterInCanID, MotorType.kBrushless);
    m_sparkEncoder = m_intakeSpark.getEncoder();
    m_intakeClosedLoopController = m_intakeSpark.getClosedLoopController();
    m_sparkEncoder.setPosition(0);

    m_wheelMotorConfig = new SparkMaxConfig();
    m_wheelMotorConfig.idleMode(IdleMode.kCoast);
    
    m_wheelMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    m_wheelMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(IntakeConstants.kIntakeWheelMinOutRange, IntakeConstants.kIntakeWheelMaxOutRange)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(IntakeConstants.kIntakeWheelMinOutRange, IntakeConstants.kIntakeWheelMaxOutRange, ClosedLoopSlot.kSlot1)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
      
      m_intakeSpark.configure(m_wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      SmartDashboard.setDefaultNumber("Intake Target Velocity", 0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command IntakeWheelsInCommand() {
    return runOnce(() -> setTargetVelocity(IntakeConstants.kIntakeInTargetVelocity));
  }

  public Command IntakeWheelsInSlowCommand() {
    return runOnce(() -> setTargetVelocity(IntakeConstants.kIntakeInSlowTargetVelocity));
  }

  public Command IntakeWheelsOutCommand() {
    return runOnce(() -> setTargetVelocity(IntakeConstants.kIntakeOutTargetVelocity));
  }

  public Command IntakeWheelsHaltCommand() {
    return runOnce(() -> setTargetVelocity(0));
  }

  public void setTargetVelocity(double targetVelocity) {
    this.targetVelocity=targetVelocity;
  }

  public boolean areIntakeWheelsSpinning() {
    if (targetVelocity==0.0)
      return false;
    else
      return true;      
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (IntakeConstants.kWheelTargetVelocityFromDashboard) {
      targetVelocity = SmartDashboard.getNumber("Intake Target Velocity", 0.0);
    }

    m_intakeClosedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);

    SmartDashboard.putNumber("INTAKE Actual Velocity", m_sparkEncoder.getVelocity());
    SmartDashboard.putNumber("INTAKE Amps", m_intakeSpark.getOutputCurrent());
    SmartDashboard.putNumber("INTAKE DutyCycle", m_intakeSpark.getAppliedOutput());
    SmartDashboard.putBoolean("INTAKE ON", areIntakeWheelsSpinning());

  }

}
