// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.armState;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.SparkLimitSwitch;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkFlex motor;
  private SparkFlexConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  //private SparkLimitSwitch m_forwardLimit;
  //private SparkLimitSwitch m_reverseLimit;

  private armState aState;
  private double targetPosition;
  private boolean motorStopped;

  // NOTE: Referencing this example code:
  //   https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java

  public ArmSubsystem() {
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor = new SparkFlex(ArmConstants.kArmInCanID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();
    encoder.setPosition(0);

     /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkFlexConfig();

    // This sets default idel mode to brake mode
    motorConfig.idleMode(IdleMode.kCoast);  

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    //m_forwardLimit = leftMotor.getForwardLimitSwitch();
    //m_reverseLimit = leftMotor.getReverseLimitSwitch();

    //m_forwardLimit. enableLimitSwitch(false);
    //m_reverseLimit.enableLimitSwitch(false);
    //SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
    //SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
    
    targetPosition = 0;
    aState=armState.ARM_RESET_POSITION;

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1) // was 0.1
        .i(0)
        .d(0)
        .outputRange(ArmConstants.MOTOR_MIN_OUT_RANGE, ArmConstants.MOTOR_MAX_OUT_RANGE)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(ArmConstants.MOTOR_MIN_OUT_RANGE, ArmConstants.MOTOR_MAX_OUT_RANGE, ClosedLoopSlot.kSlot1)
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


    motorStopped=false;
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("ARM Target Position", 0);
    SmartDashboard.setDefaultBoolean("ARM Reset Encoder", false);

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

  public Command SetArmUpCommand() {
    setArmState(armState.ARM_UP_POSITION);
    return runOnce(() -> setTargetPosition(aState.getPosition()));
  }

  public Command SetArmDownCommand() {
    setArmState(armState.ARM_DOWN_POSITION);
    return runOnce(() -> setTargetPosition(aState.getPosition()));  
  }

  public Command SetArmMidCommand() {
    setArmState(armState.ARM_MID_POSITION);
    return runOnce(() -> setTargetPosition(aState.getPosition()));  
  }

  public Command DisableArmMotorCommand() {
    return runOnce(() -> DisableArmMotor());  
  }
  
  public Command EnableArmMotorCommand() {
    return runOnce(() -> EnableArmMotor());  
  }

  public void DisableArmMotor() {
    motorStopped=true;
    motor.disable();
    setArmState(armState.UNKNOWN);
  }

  public void EnableArmMotor() {
    motorStopped=false;
  }

  public armState getArmState() {
    return this.aState;
  }

  public void setArmState(armState aState) {
    this.aState = aState;
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition=targetPosition;
  }

  public double getArmActualPosition() {
   return encoder.getPosition();
  }

  public void resetArmPosition() {
    encoder.setPosition(0);
  }

  public void setIdleModeToCoast() {
    motorConfig.idleMode(IdleMode.kCoast);
  }

  public void setIdleModeToBrake() {
    motorConfig.idleMode(IdleMode.kBrake);
  }

  public boolean isArmMotorEnabled() {
    if (motorStopped)
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
  
    if (ArmConstants.kArmTargetPositionFromDashboard) {
      targetPosition = SmartDashboard.getNumber("ARM Target Position", 0);
    }
    if (!motorStopped) {
      closedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
  
    // Display encoder position and velocity
    SmartDashboard.putNumber("ARM Target Position", targetPosition);
    SmartDashboard.putNumber("ARM Actual Position", encoder.getPosition());
    SmartDashboard.putNumber("ARM Amps", motor.getOutputCurrent());
    SmartDashboard.putNumber("ARM DutyCycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("ARM Speed", motor.get());
    SmartDashboard.putBoolean("ARM Active", isArmMotorEnabled());

    //SparkLimitSwitch forwardLimitSwitch = leftMotor.getForwardLimitSwitch();
    //SmartDashboard.putBoolean("ELEV Left Limit FWD", forwardLimitSwitch.isPressed());
    //SparkLimitSwitch reverseLimitSwitch = leftMotor.getForwardLimitSwitch();
    //SmartDashboard.putBoolean("ELEV Left Limit REV", reverseLimitSwitch.isPressed());

  }

  public double getPosition () { 
    return encoder.getPosition(); 
  }

}
