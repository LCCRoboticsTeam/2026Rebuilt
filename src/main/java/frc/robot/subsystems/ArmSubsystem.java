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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.SparkLimitSwitch;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax leftMotor;
  private SparkMaxConfig leftMotorConfig;
  private SparkClosedLoopController leftClosedLoopController;
  private RelativeEncoder leftEncoder;
  //private SparkLimitSwitch m_forwardLimit;
  //private SparkLimitSwitch m_reverseLimit;

  private SparkMax rightMotor;

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
    leftMotor = new SparkMax(ArmConstants.kLeftArmInCanID, MotorType.kBrushless);
    leftClosedLoopController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPosition(0);

     /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    leftMotorConfig = new SparkMaxConfig();

    // This sets default idel mode to brake mode
    leftMotorConfig.idleMode(IdleMode.kBrake);

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    leftMotorConfig.encoder
        .positionConversionFactor(3)
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
    leftMotorConfig.closedLoop
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
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Right Motor is the follower
    rightMotor = new SparkMax(ArmConstants.kRightArmInCanID, MotorType.kBrushless);

    rightMotor.configure(leftMotorConfig.follow(leftMotor, true), ResetMode.kResetSafeParameters, 
      PersistMode.kNoPersistParameters);

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
    return runOnce(() -> setTargetPosition(ArmConstants.kArmUpPosition));
  }

  public Command SetArmMidCommand() {
    setArmState(armState.ARM_MID_POSITION);
    return runOnce(() -> setTargetPosition(ArmConstants.kArmMidPosition));
  }

  public Command SetArmDownCommand() {
    setArmState(armState.ARM_DOWN_POSITION);
    return runOnce(() -> setTargetPosition(ArmConstants.kArmDownPosition));
  }

  public Command DisableArmMotorCommand() {
    return runOnce(() -> DisableArmMotor());  
  }
  
  public Command EnableArmMotorCommand() {
    return runOnce(() -> EnableArmMotor());  
  }

  public void DisableArmMotor() {
    motorStopped=true;
    leftMotor.disable();
    //setArmState(armState.UNKNOWN);
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
   return leftEncoder.getPosition();
  }

  public void resetArmPosition() {
    leftEncoder.setPosition(0);
  }

  public void setIdleModeToCoast() {
    leftMotorConfig.idleMode(IdleMode.kCoast);
  }

  public void setIdleModeToBrake() {
    leftMotorConfig.idleMode(IdleMode.kBrake);
  }

  public boolean isArmMotorEnabled() {
    if (motorStopped)
      return false;
    else
      return true;      
  }

  public boolean isArmDown() {
    if (getArmState()==armState.ARM_DOWN_POSITION)
      return true;
    else
      return false;      
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return true;
  }


  @Override
  public void periodic() {
  
    if (ArmConstants.kArmTargetPositionFromDashboard) {
      targetPosition = SmartDashboard.getNumber("ARM Target Position", 0);
    }
    if (!motorStopped) {
      leftClosedLoopController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
  
    // Display encoder position and velocity
    SmartDashboard.putNumber("ARM Target Position", targetPosition);
    SmartDashboard.putNumber("ARM Actual Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("ARM Left Amps", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("ARM Left DutyCycle", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("ARM Right Amps", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("ARM Right DutyCycle", rightMotor.getAppliedOutput());
    //SmartDashboard.putNumber("ARM Speed", leftMotor.get());
    SmartDashboard.putBoolean("ARM Active", isArmMotorEnabled());


    //SparkLimitSwitch forwardLimitSwitch = leftMotor.getForwardLimitSwitch();
    //SmartDashboard.putBoolean("ELEV Left Limit FWD", forwardLimitSwitch.isPressed());
    //SparkLimitSwitch reverseLimitSwitch = leftMotor.getForwardLimitSwitch();
    //SmartDashboard.putBoolean("ELEV Left Limit REV", reverseLimitSwitch.isPressed());

  }

  public double getPosition () { 
    return leftEncoder.getPosition(); 
  }

}
