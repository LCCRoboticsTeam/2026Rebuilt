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
  private final IntakeWheelsSubsystem intakeWheelsSubsystem;
  //private final ArmSubsystem armSubsystem;
  private final ShooterInSubsystem shooterInSubsystem;
  private final ShooterOutSubsystem shooterOutSubsystem;
  private final ClimberSubsystem climberSubsystem;
  //private final GyroSubsystem gyroSubsystem;

  // The driver's controllers
  private final XboxController driverXboxController = new XboxController(OIConstants.kDriverControllerPort); 
  private final CommandXboxController driverCommandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController manipulatorCommandXboxController = new CommandXboxController(OIConstants.kManipulatorControllerPort);
  //2025 REEFSCAPE - private final CommandLaunchpadController commandLaunchpad = new CommandLaunchpadController(OIConstants.kLaunchpadControllerPort);

  // Dashboard - Choosers
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(BooleanSupplier isRobotEnabled) {
    driveSubsystem = new DriveSubsystem();
    intakeWheelsSubsystem = new IntakeWheelsSubsystem();
    //armSubsystem = new ArmSubsystem();
    shooterInSubsystem = new ShooterInSubsystem();
    shooterOutSubsystem = new ShooterOutSubsystem();
    climberSubsystem = new ClimberSubsystem();
    //gyroSubsystem = new GyroSubsystem();

    // We always start with CLIMBER_DOWN and the Ratchet disabled
    climberSubsystem.setClimberState(ClimberState.CLIMBER_DOWN);
    climberSubsystem.setServoAngle(ClimberConstants.kServoAngleToDisableRatchet);

    // Register Named Commands
    //   These are simple commands that are directly generated from the subsystem itself
    NamedCommands.registerCommand("IntakeIn", intakeWheelsSubsystem.IntakeWheelsInCommand());
    NamedCommands.registerCommand("IntakeOut", intakeWheelsSubsystem.IntakeWheelsOutCommand());
    NamedCommands.registerCommand("IntakeHalt", intakeWheelsSubsystem.IntakeWheelsHaltCommand());
    //NamedCommands.registerCommand("ArmUp", armSubsystem.SetArmUpCommand());
    //NamedCommands.registerCommand("ArmDown", armSubsystem.SetArmDownCommand());
    //NamedCommands.registerCommand("ArmMid", armSubsystem.SetArmMidCommand());
    NamedCommands.registerCommand("ShooterInForward",shooterInSubsystem.Forward());
    NamedCommands.registerCommand("ShooterInHalt", shooterInSubsystem.Halt());
    NamedCommands.registerCommand("ShooterInReversed", shooterInSubsystem.Reversed());
    NamedCommands.registerCommand("ShooterOutForward", shooterOutSubsystem.Forward());
    NamedCommands.registerCommand("ShooterOutHalt", shooterOutSubsystem.Halt());
    NamedCommands.registerCommand("ShooterOutReversed", shooterOutSubsystem.Reversed());
    NamedCommands.registerCommand("ClimbUp", new MoveClimberUpCommand(climberSubsystem));
    NamedCommands.registerCommand("ClimbDown", new MoveClimberDownCommand(climberSubsystem));

    //   These are class-based commands
    NamedCommands.registerCommand("SwerveSlideRight", new SwerveSlideCommand(driveSubsystem, true, DriveConstants.kSwerveSlideSpeed));
    NamedCommands.registerCommand("SwerveSlideLeft", new SwerveSlideCommand(driveSubsystem, false, DriveConstants.kSwerveSlideSpeed));
    //NamedCommands.registerCommand("JostleArm", new JostleArmCommand(armSubsystem));
    NamedCommands.registerCommand("StartShooter",new SequentialCommandGroup(NamedCommands.getCommand("ShooterOutForward"), 
                                                                                 new WaitCommand(0.5), 
                                                                                 NamedCommands.getCommand("ShooterInForward")));
    NamedCommands.registerCommand("StopShooter", new SequentialCommandGroup(NamedCommands.getCommand("ShooterInHalt"), 
                                                                                 new WaitCommand(0.5), 
                                                                                 NamedCommands.getCommand("ShooterOutHalt")));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("MoveOut2M");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    
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
      SmartDashboard.putData("ArmUp", NamedCommands.getCommand("ArmUp"));
      SmartDashboard.putData("ArmDown", NamedCommands.getCommand("ArmDown"));
      SmartDashboard.putData("ArmMid", NamedCommands.getCommand("ArmMid"));
    }
    if (ShooterConstants.kShooterCommandsFromDashboard) {
      SmartDashboard.putData("ShooterInForward",NamedCommands.getCommand("ShooterInForward"));
      SmartDashboard.putData("ShooterInHalt", NamedCommands.getCommand("ShooterInHalt"));
      SmartDashboard.putData("ShooterInReversed", NamedCommands.getCommand("ShooterInReversed"));
      SmartDashboard.putData("ShooterOutForward", NamedCommands.getCommand("ShooterOutForward"));
      SmartDashboard.putData("ShooterOutHalt", NamedCommands.getCommand("ShooterOutHalt"));
      SmartDashboard.putData("ShooterOutReversed", NamedCommands.getCommand("ShooterOutReversed"));
    }
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

    // MANIPULATOR XBOX Controller
    manipulatorCommandXboxController.y().whileTrue(NamedCommands.getCommand("StartShooter"));  // Hold button to keep shooting
    manipulatorCommandXboxController.y().negate().onTrue(NamedCommands.getCommand("StopShooter"));
    manipulatorCommandXboxController.a().whileTrue(NamedCommands.getCommand("IntakeIn")); // Hold button to keep intaking
    manipulatorCommandXboxController.a().negate().onTrue(NamedCommands.getCommand("IntakeHalt"));
    manipulatorCommandXboxController.back().onTrue(NamedCommands.getCommand("ClimbUp"));
    manipulatorCommandXboxController.start().onTrue(NamedCommands.getCommand("ClimbDown"));
    //manipulatorCommandXboxController.b().onTrue(NamedCommands.getCommand("ArmMid"));
    //manipulatorCommandXboxController.x().onTrue(NamedCommands.getCommand("ArmDown"));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return null;
  }

  public void zeroHeading() {
    driveSubsystem.zeroHeading();
  }
}
