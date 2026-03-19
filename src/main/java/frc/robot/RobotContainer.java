// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

// Cameras and Vision
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
//import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems defined here...
  private final DriveSubsystem driveSubsystem;
  private final IntakeWheelsSubsystem intakeWheelsSubsystem = new IntakeWheelsSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ShooterInSubsystem shooterInSubsystem = new ShooterInSubsystem();
  private final ShooterOutSubsystem shooterOutSubsystem = new ShooterOutSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  //private final GyroSubsystem gyroSubsystem = new GyroSubsystem();

  // The driver's controllers
  private final XboxController driverXboxController = new XboxController(OIConstants.kDriverControllerPort); 
  private final CommandXboxController driverCommandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController manipulatorCommandXboxController = new CommandXboxController(OIConstants.kManipulatorControllerPort);
  //2025 REEFSCAPE - private final CommandLaunchpadController commandLaunchpad = new CommandLaunchpadController(OIConstants.kLaunchpadControllerPort);

  // Dashboard - Choosers
  private final SendableChooser<Command> autoChooser;

  // Cameras and Vision
  //UsbCamera hoppersideUsbCamera = CameraServer.startAutomaticCapture(1);
  //UsbCamera frontsideUsbCamera = CameraServer.startAutomaticCapture(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(BooleanSupplier isRobotEnabled) {
    driveSubsystem = new DriveSubsystem();

    // We always start with CLIMBER_DOWN and the Ratchet disabled
    climberSubsystem.setClimberState(ClimberState.CLIMBER_DOWN);
    climberSubsystem.setServoAngle(ClimberConstants.kServoAngleToDisableRatchet);

    // Register Named Commands
    //   These are simple commands that are directly generated from the subsystem itself
    NamedCommands.registerCommand("IntakeIn", intakeWheelsSubsystem.IntakeWheelsInCommand());
    NamedCommands.registerCommand("IntakeInSlow", intakeWheelsSubsystem.IntakeWheelsInSlowCommand());
    NamedCommands.registerCommand("IntakeOut", intakeWheelsSubsystem.IntakeWheelsOutCommand());
    NamedCommands.registerCommand("IntakeHalt", intakeWheelsSubsystem.IntakeWheelsHaltCommand());
    NamedCommands.registerCommand("ArmUp", armSubsystem.SetArmUpCommand());
    NamedCommands.registerCommand("ArmDown", armSubsystem.SetArmDownCommand());
    NamedCommands.registerCommand("ArmMid", armSubsystem.SetArmMidCommand());
    NamedCommands.registerCommand("DisableArmMotor", armSubsystem.DisableArmMotorCommand());
    NamedCommands.registerCommand("EnableArmMotor", armSubsystem.EnableArmMotorCommand());
    NamedCommands.registerCommand("ShooterInForward",shooterInSubsystem.Forward());
    NamedCommands.registerCommand("ShooterInHalt", shooterInSubsystem.Halt());
    NamedCommands.registerCommand("ShooterInReversed", shooterInSubsystem.Reversed());
    NamedCommands.registerCommand("ShooterInCommand", new ShooterInCommand(shooterInSubsystem));
    NamedCommands.registerCommand("ShooterOutForward", shooterOutSubsystem.ForwardLow());
    NamedCommands.registerCommand("ShooterOutForwardHigh", shooterOutSubsystem.ForwardHigh());
    NamedCommands.registerCommand("ShooterOutHalt", shooterOutSubsystem.Halt());
    NamedCommands.registerCommand("ShooterOutReversed", shooterOutSubsystem.Reversed());
    NamedCommands.registerCommand("ShooterOutCommand", new ShooterOutCommand(shooterOutSubsystem));

    //   These are class-based or more complex commands
    NamedCommands.registerCommand("ClimbUp", new MoveClimberUpCommand(climberSubsystem));
    NamedCommands.registerCommand("ClimbDown", new MoveClimberDownCommand(climberSubsystem));
    NamedCommands.registerCommand("SwerveSlideRight", new SwerveSlideCommand(driveSubsystem, true, DriveConstants.kSwerveSlideSpeed));
    NamedCommands.registerCommand("SwerveSlideLeft", new SwerveSlideCommand(driveSubsystem, false, DriveConstants.kSwerveSlideSpeed));
    NamedCommands.registerCommand("SwerveRotateCommandRight", new SwerveRotateCommand(driveSubsystem, true, DriveConstants.kSwerveRotateSpeed));
    NamedCommands.registerCommand("SwerveRotateCommandLeft", new SwerveRotateCommand(driveSubsystem, false, DriveConstants.kSwerveRotateSpeed));

    NamedCommands.registerCommand("DropArm",new SequentialCommandGroup(NamedCommands.getCommand("ArmDown"), 
                                                                            new WaitCommand(1.0),
                                                                            NamedCommands.getCommand("DisableArmMotor")));
    NamedCommands.registerCommand("JostleArm",new SequentialCommandGroup(NamedCommands.getCommand("IntakeInSlow"),
                                                                              NamedCommands.getCommand("EnableArmMotor"),
                                                                              NamedCommands.getCommand("ArmMid"), 
                                                                              new WaitCommand(0.8),
                                                                              NamedCommands.getCommand("IntakeHalt"),
                                                                              NamedCommands.getCommand("ArmDown"),
                                                                              new WaitCommand(0.5),
                                                                              NamedCommands.getCommand("DisableArmMotor")));
    NamedCommands.registerCommand("RaiseArm",new SequentialCommandGroup(NamedCommands.getCommand("IntakeInSlow"),
                                                                            NamedCommands.getCommand("ArmUp"), 
                                                                            new WaitCommand(1.5),
                                                                            NamedCommands.getCommand("DisableArmMotor")));                                                                  
    NamedCommands.registerCommand("StartShooter",new SequentialCommandGroup(NamedCommands.getCommand("ShooterOutForward"), 
                                                                                 new WaitCommand(0.5), 
                                                                                 NamedCommands.getCommand("ShooterInForward")));
    NamedCommands.registerCommand("StartShooterHigh",new SequentialCommandGroup(NamedCommands.getCommand("ShooterOutForwardHigh"), 
                                                                                 new WaitCommand(0.75), 
                                                                                 NamedCommands.getCommand("ShooterInForward")));
    NamedCommands.registerCommand("StopShooter", new SequentialCommandGroup(NamedCommands.getCommand("ShooterInHalt"), 
                                                                                  new WaitCommand(0.5), 
                                                                                  NamedCommands.getCommand("ShooterOutHalt")));
    //NamedCommands.registerCommand("ShooterToggleCommand", new SequentialCommandGroup(NamedCommands.getCommand("ShooterOutCommand"),
    //                                                                              new WaitCommand(0.5),
    //                                                                              NamedCommands.getCommand("ShooterInCommand")));

    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser("MoveOut2M");
    autoChooser = AutoBuilder.buildAutoChooser("RightHubShoot");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Reset Gyro Heading", driveSubsystem.zeroHeadingCommand());
    
    // Configure default commands
    driveSubsystem.setDefaultCommand(new SwerveGamepadDriveCommand(driveSubsystem, driverCommandXboxController::getLeftX,
      driverCommandXboxController::getLeftY, driverCommandXboxController::getRightX,  
      driverXboxController::getLeftStickButton));
    if (IntakeConstants.kIntakeCommandsFromDashboard) {
      SmartDashboard.putData("IntakeIn", NamedCommands.getCommand("IntakeIn"));
      SmartDashboard.putData("IntakeOut", NamedCommands.getCommand("IntakeOut"));
      SmartDashboard.putData("IntakeHalt", NamedCommands.getCommand("IntakeHalt"));
    }
    if (ArmConstants.kArmCommandsFromDashboard) {
      SmartDashboard.putData("DropArm", NamedCommands.getCommand("DropArm"));
      SmartDashboard.putData("JostleArm", NamedCommands.getCommand("JostleArm"));
      SmartDashboard.putData("RaiseArm", NamedCommands.getCommand("RaiseArm"));
      SmartDashboard.putData("DisableArmMotor", NamedCommands.getCommand("DisableArmMotor"));
    }
    if (ShooterConstants.kShooterCommandsFromDashboard) {
      SmartDashboard.putData("ShooterInForward",NamedCommands.getCommand("ShooterInForward"));
      SmartDashboard.putData("ShooterInHalt", NamedCommands.getCommand("ShooterInHalt"));
      SmartDashboard.putData("ShooterInReversed", NamedCommands.getCommand("ShooterInReversed"));
      SmartDashboard.putData("ShooterOutForward", NamedCommands.getCommand("ShooterOutForward"));
      SmartDashboard.putData("ShooterOutHalt", NamedCommands.getCommand("ShooterOutHalt"));
      SmartDashboard.putData("ShooterOutReversed", NamedCommands.getCommand("ShooterOutReversed"));
    }

    //SmartDashboard.putData("SwerveRotateCommandRight", NamedCommands.getCommand("SwerveRotateCommandRight"));
    //SmartDashboard.putData("SwerveRotateCommandLeft", NamedCommands.getCommand("SwerveRotateCommandLeft"));
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // XBOX Controller Diagram
    //   https://gist.github.com/palmerj/586375bcc5bc83ccdaf00c6f5f863e86

    // DRIVER XBOX Controller
    //   Note: Right stick and Left stick already mapped via SwerveGamepadDriveCommand() in earlier code
    driverCommandXboxController.rightBumper().whileTrue(NamedCommands.getCommand("SwerveSlideRight"));
    driverCommandXboxController.leftBumper().whileTrue(NamedCommands.getCommand("SwerveSlideLeft"));
    // Arm Related
    driverCommandXboxController.povDown().onTrue(NamedCommands.getCommand("DropArm"));
    driverCommandXboxController.povUp().onTrue(NamedCommands.getCommand("ArmUp"));
    //driverCommandXboxController.b().onTrue(NamedCommands.getCommand("SwerveRotateCommandRight"));
    //driverCommandXboxController.x().onTrue(NamedCommands.getCommand("SwerveRotateCommandLeft"));

    // MANIPULATOR XBOX Controller
    //  Shooting Related
    manipulatorCommandXboxController.y().whileTrue(NamedCommands.getCommand("StartShooter"));  // Hold button to keep shooting
    manipulatorCommandXboxController.y().negate().onTrue(NamedCommands.getCommand("StopShooter"));
    manipulatorCommandXboxController.b().whileTrue(NamedCommands.getCommand("StartShooterHigh"));  // Hold button to keep shooting
    manipulatorCommandXboxController.b().negate().onTrue(NamedCommands.getCommand("StopShooter"));
    manipulatorCommandXboxController.povUp().onTrue(NamedCommands.getCommand("JostleArm"));
    //  Intake Related
    manipulatorCommandXboxController.a().onTrue(new ConditionalCommand(NamedCommands.getCommand("IntakeHalt"), 
                                                                      NamedCommands.getCommand("IntakeIn"), 
                                                                      intakeWheelsSubsystem::areIntakeWheelsSpinning));
    manipulatorCommandXboxController.x().onTrue(NamedCommands.getCommand("IntakeOut"));
    manipulatorCommandXboxController.x().negate().onTrue(NamedCommands.getCommand("IntakeHalt"));
    // Climber Related
    manipulatorCommandXboxController.back().onTrue(NamedCommands.getCommand("ClimbUp"));
    manipulatorCommandXboxController.start().onTrue(NamedCommands.getCommand("ClimbDown"));
    // Arm Related
    manipulatorCommandXboxController.leftBumper().onTrue(NamedCommands.getCommand("JostleArm"));

    //manipulatorCommandXboxController.rightTrigger().onTrue(NamedCommands.getCommand("ShooterToggleCommand"));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Do we need to program driveSubsystem to tell it which way the robot is facing?
    // driveSubsystem.robotFacingDriveStation();
    return autoChooser.getSelected();
    //return null;
  }

  public void zeroHeading() {
    driveSubsystem.zeroHeading();
  }
}
