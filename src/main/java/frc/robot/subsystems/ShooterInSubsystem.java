// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.motorState;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

public class ShooterInSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private double motorTargetVelocity;
  private motorState mState;

  public ShooterInSubsystem() {
    motor = new SparkMax(ShooterConstants.kShooterInCanID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kCoast);

    motorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1)
    ;

    motorTargetVelocity= 0;
    mState=motorState.UNKNOWN;
    
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(ShooterConstants.kMotorInMinOutRange, ShooterConstants.kMotorInMaxOutRange)
      .p(0.0001, ClosedLoopSlot.kSlot1)
      .i(0,ClosedLoopSlot.kSlot1)
      .d(0,ClosedLoopSlot.kSlot1)
      .outputRange(ShooterConstants.kMotorInMinOutRange, ShooterConstants.kMotorInMaxOutRange, ClosedLoopSlot.kSlot1)
      .feedForward
        .kV(12.0/5767,ClosedLoopSlot.kSlot1);
    
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SmartDashboard.setDefaultNumber("Shooter In Target Velocity", 0);
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
  
  public Command Halt() {
    return runOnce(() -> setmotorTargetVelocity(0));
  }

  public Command Forward() {
    return runOnce(() -> setmotorTargetVelocity(ShooterConstants.kInForward));
  }
  
  public Command Reversed() {
    return runOnce(() -> setmotorTargetVelocity(ShooterConstants.kInReversed));
  }
  
  public void setmotorTargetVelocity(double motorTargetVelocity) {
    this.motorTargetVelocity=motorTargetVelocity;
  }

  public motorState getMotorState() {
    return this.mState;
  }

  public void setMotorState(motorState mState) {
    this.mState =  mState;
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
    if (ShooterConstants.kMotorTargetVelocityFromDashboard) {
      double targetVelocity = SmartDashboard.getNumber("Shooter In Target Velocity", 0);
        closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }
    else {
      closedLoopController.setSetpoint(motorTargetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }
    SmartDashboard.putNumber("Shooter In Actual Vel", encoder.getVelocity());
    SmartDashboard.putNumber("Shooter In Amps", motor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter In DutyCycle", motor.getAppliedOutput());
  }

}
