// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.Climber.VortexClimberDown;
import frc.robot.commands.Climber.VortexClimberPIDCommand;
import frc.robot.commands.Climber.VortexClimberUp;
import frc.robot.commands.Elevator.VortexElevatorDown;
import frc.robot.commands.Elevator.VortexElevatorPIDCommand;
import frc.robot.commands.Intake.GoUntilBeamBreak;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeGo;
import frc.robot.commands.Shooter.FlywheelSpinCommand;
import frc.robot.commands.Shooter.PivotCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VortexClimberSub;
import frc.robot.subsystems.VortexElevatorSub;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */


//Ctrl+F & looking for keywords can help find potential issues in the code or what needs to be changed. ex IDK, Change, etc.


 public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FlywheelSubsystem m_FlywheelSubsystem = new FlywheelSubsystem();
  // To fix on Monday
  public ShooterPositions position;
  private final PivotSubsystem m_PivotSubsystem= new PivotSubsystem(position);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final VortexClimberSub m_ClimberSub = new VortexClimberSub();
  private final VortexElevatorSub m_ElevatorSub = new VortexElevatorSub();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(1);

  private final CommandXboxController driver = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    // Flywheel Bindings
    //driverController.rightBumper().onTrue(new FlywheelSpinCommand(m_FlywheelSubsystem, 1000)); // finished
    // Pivot Bindings
    //driverController.leftBumper().onTrue(new PivotCommand(m_PivotSubsystem, 100));

    //driverController.a().onTrue(new GoUntilBeamBreak(m_IntakeSubsystem));// finished :)
    driverController.b().onTrue(new VortexClimberPIDCommand(m_ClimberSub));
    driverController.x().onTrue(new VortexElevatorPIDCommand(m_ElevatorSub));
    driverController.a().onTrue(new VortexElevatorDown(m_ElevatorSub));
    // Climber Bindings
    //driverController.x().onTrue(new VortexClimberDown(null/*?????*/));
    //driverController.y().onTrue(new VortexClimberUp(null/*?????*/));
    //driverController.leftBumper().whileTrue(new IntakeCommand(flywheelSubsystem, 0.3));
    //driverController.rightBumper().whileTrue(new IntakeCommand(flywheelSubsystem, -0.3));
    //driver.button(0, new FlywheelSpinCommand(null, driver));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null; // CHANGE LATER!!
  }
}
