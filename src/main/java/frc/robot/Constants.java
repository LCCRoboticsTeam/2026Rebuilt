// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // -------------------- LED --------------------
  public static final class LEDConstants {
    public static final int PWM_PORT = 1;
    // You can find full list of LED color support at: 
    //   https://1166281274-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F-ME3KPEhFI6-MDoP9nZD%2Fuploads%2FMOYJvZmWgxCVKJhcV5fn%2FREV-11-1105-LED-Patterns.pdf?alt=media&token=e8227890-6dd3-498d-834a-752fa43413fe 
    
    public static final double SOLID_LAWN_GREEN = 0.71;
    public static final double SOLID_LIME_GREEN = 0.73;

    public static final double SOLID_DARK_GREEN = 0.75;
    public static final double SOLID_GREEN = 0.77;
    public static final double SOLID_BLUE_GREEN = 0.79;
    public static final double SOLID_AQUA_BLUE = 0.81;
    public static final double SOLID_SKY_BLUE = 0.83;
    public static final double SOLID_BLUE = 0.85;
    public static final double SOLID_DARK_BLUE = 0.87;
    public static final double SOLID_BLUE_VIOLET = 0.89;
    public static final double SOLID_WHITE = 0.93;

    public static final double COLOR_WAVES_OCEAN_PALETTE = -0.41;
    public static final double TWINKLE_OCEAN_PALETTE = -0.51;
    public static final double BEATS_PER_MIN_OCEAN_PALETTE = -0.65;
    public static final double SINELON_OCEAN_PALETTE = -0.75;
    public static final double RAINBOW_OCEAN_PALETTE = -0.95;

    public static final double FIXED_PALETTE_PATTERN_FIRE_MEDIUM = -0.59;
    public static final double FIXED_PALETTE_PATTERN_FIRE_LARGE = -0.57;
  }
  // ---------------------------------------------

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.75;  // Orig 4.8, 2024Crecendo it was 4.0, 2025Reefscape trying slower
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionalSlewRate = 4; // radians per second; was .6
    public static final double kMagnitudeSlewRate = 3; // percent per second (1 = 100%); was .9
    public static final double kRotationalSlewRate = 3; // percent per second (1 = 100%); was .9

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.25);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.25); 
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 16;
    public static final int kRearRightTurningCanId = 18;

    public static final boolean kGyroReversed = false;

    public static final double kSwerveSlideSpeed = 0.15;
    public static final double kSwerveAutoAlignSlideSpeed = 0.15;  // Not as reliable if faster

    public static final double kSwerveBackupSpeed = 0.4;
    public static final double kSwerveRotateRightSpeed = 0.75;
    public static final double kSwerveRotateLeftSpeed = -0.75;

    public static final int kSwerveBackupCommandRuntimeInMs = 400;
    public static final int kSwerveRotateCommandRuntimeInMs = 50;
    public static final int kSwerveSlideCommandRuntimeInMs = 1400;

    public static final boolean usePhotonPoseEstimator = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0731; // was 0.0741; was 0.0762 for 2024 Crescendo (was 0.0762)
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final int kLaunchpadControllerPort = 2;
    public static final double kDriveDeadband = 0.065;  // was 0.05, 2024Crescendo used 0.065

  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

// -------------------- CLIMBER --------------------
  public final class ClimberConstants {
    public static final int kClimberCanID = 8;
    public static final int kClimberPositionUp = 68; // 68 is the absolute max
    public static final int kClimberPositionDown = -2; // -2 is the absolute lowest 

    public static final double kmaxOutRange = 0.5;
    public static final double kminOutRange = -0.5;

    public static final double kServoAngleToEnableRatchet = 0.0;
    public static final double kServoAngleToDisableRatchet = 15.0;

    public static final boolean kTargetPositionFromDashboard = false;
    public static final boolean kServoAngleFromDashboard = false;
  }
  public enum ClimberState {
    UNKNOWN,
    CLIMBER_UP,
    CLIMBER_DOWN
  }
  // ---------------------------------------------

  // -------------------- SHOOTER --------------------

  public static final class ShooterConstants {
    public static final boolean kMotorTargetVelocityFromDashboard = false;
    public static final boolean kShooterCommandsFromDashboard = true;
    public static final int kShooterInCanID = 3;
    public static final int kShooterOutCanID = 4;

    public static final double kMotorInMaxOutRange = 0.8;
    public static final double kMotorInMinOutRange = -0.8;
    public static final double kInForward = -1600;
    public static final double kInReversed = 900;

    public static final double kMotorOutMaxOutRange = 0.8;
    public static final double kMotorOutMinOutRange = -0.8; 
    public static final double kOutForward = 3000;
    public static final double kOutReversed = -900;
  } 

  // ---------------------------------------------

  // -------------------- INTAKE --------------------

  public static final class IntakeConstants {
    public static final boolean kWheelTargetVelocityFromDashboard = false;
    public static final boolean kIntakeCommandsFromDashboard = false;
    public static final int kShooterInCanID = 5;
    public static final double kIntakeWheelMaxOutRange = 0.7;
    public static final double kIntakeWheelMinOutRange = -0.7;
    public static final double kIntakeInTargetVelocity = 1500;
    public static final double kIntakeOutTargetVelocity = -900;
  }
  // ---------------------------------------------

  public enum motorState {
    UNKNOWN,
    RUNNING,
    STOPPED;
  }

  public static final class ArmConstants {
    public static final boolean kArmTargetPositionFromDashboard = true;
    public static final boolean kArmCommandsFromDashboard = true;
    public static final int kArmInCanID = 10;

    public static final double MOTOR_MIN_OUT_RANGE = -0.1;
    public static final double MOTOR_MAX_OUT_RANGE = 0.1;
  }

  public enum armState {
    UNKNOWN,
    ARM_UP_POSITION(0),
    ARM_MID_POSITION(90),
    ARM_DOWN_POSITION(180);

    private double armPosition;
    armState(double armPosition) {
      this.armPosition = armPosition;
    }
    armState() {}
    public double getPosition() {
      return armPosition;
    }
  }

}

