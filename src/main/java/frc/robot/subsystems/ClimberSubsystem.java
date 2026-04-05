// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.wpilibj.Servo;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private double targetPosition;

  private ClimberState climberState;

  private Servo servo;

  public ClimberSubsystem() {
    motor = new SparkMax(ClimberConstants.kClimberCanID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();
    motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kBrake);

    targetPosition = 0;
    climberState = ClimberState.UNKNOWN;
    encoder.setPosition(0);

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */

      motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(ClimberConstants.kminOutRange, ClimberConstants.kmaxOutRange)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(ClimberConstants.kminOutRange, ClimberConstants.kmaxOutRange, ClosedLoopSlot.kSlot1)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    servo = new Servo(0);
    SmartDashboard.setDefaultNumber("CLMB Servo Tgt Angle",0);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("CLMB Target Pos", 0);
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition=targetPosition;
  }

  public double getActualPosition() {
   return encoder.getPosition();
  }

  public void resetPosition() {
     // Reset the encoder position to 0
     encoder.setPosition(0);
  }

   public ClimberState getClimberState() {
    return this.climberState;
  }

  public void setClimberState(ClimberState climberState) {
    this.climberState = climberState;
  }

  public boolean isClimberUp() {
    if (this.climberState==ClimberState.CLIMBER_UP)
      return true;
    else
      return false;
  }

  public void setServoAngle (double angle) {
    servo.setAngle(angle);
  }
  public double getServoAngle () {
    return servo.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      if (ClimberConstants.kTargetPositionFromDashboard) 
        targetPosition = SmartDashboard.getNumber("CLMB Target Pos", 0);

      // Since we reset to Postion 0, which is when the climber is down,
      // we should NEVER allow a position that is negative.
      if (targetPosition>=-2) {  
        // NOTE: The actual value for position to result in the climber to go UP is NEGATIVE,
        // Thus we will make the value passed in setReferene a negative number
        closedLoopController.setSetpoint(-1*targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      }
      SmartDashboard.putNumber("CLMB Actual Pos", encoder.getPosition());
      SmartDashboard.putNumber("CLMB Amps", motor.getOutputCurrent());
      SmartDashboard.putNumber("CLMB DutyCycle", motor.getAppliedOutput());

      if (ClimberConstants.kServoAngleFromDashboard) 
        setServoAngle(SmartDashboard.getNumber("CLMB Servo Tgt Angle", 0));
      SmartDashboard.putNumber("CLMB Servo Act Angle", getServoAngle());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}