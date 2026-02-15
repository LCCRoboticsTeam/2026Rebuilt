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
  private final ShooterInSubsystem shooterInSubsystem;
  private final ShooterOutSubsystem ShooterOutSubsystem;

  // The driver's controllers
  private final XboxController driverXboxController = new XboxController(OIConstants.kDriverControllerPort); 
  private final CommandXboxController driverCommandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);

  //2025 REEFSCAPE - private final CommandXboxController manipulatorCommandXboxController = new CommandXboxController(OIConstants.kManipulatorControllerPort);

  //2025 REEFSCAPE - private final CommandLaunchpadController commandLaunchpad = new CommandLaunchpadController(OIConstants.kLaunchpadControllerPort);

  // Dashboard - Choosers
  //2025 REEFSCAPE - private final SendableChooser<Command> autoChooser;
  //2025 REEFSCAPE - private final SendableChooser<Command> reefAlgaeChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(BooleanSupplier isRobotEnabled) {
    driveSubsystem = new DriveSubsystem();
    shooterInSubsystem = new ShooterInSubsystem();
    ShooterOutSubsystem = new ShooterOutSubsystem();

    // Configure the trigger bindings
    configureBindings();
    
    // Configure default commands
    driveSubsystem.setDefaultCommand(new SwerveGamepadDriveCommand(driveSubsystem, driverCommandXboxController::getLeftX,
      driverCommandXboxController::getLeftY, driverCommandXboxController::getRightX,  
      driverXboxController::getLeftStickButton));

    SmartDashboard.putData("Shooter In Forward",shooterInSubsystem.Forward());
    SmartDashboard.putData("Shooter In Halt", shooterInSubsystem.Halt());
    SmartDashboard.putData("Shooter In Reversed", shooterInSubsystem.Reversed());
    SmartDashboard.putData("Shooter Out Forward", ShooterOutSubsystem.Forward());
    SmartDashboard.putData("Shooter Out Halt", ShooterOutSubsystem.Halt());
    SmartDashboard.putData("Shooter Out Reversed", ShooterOutSubsystem.Reversed());
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
