// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.Climber.ClimberDownCommand;
import frc.robot.commands.Climber.ClimberPIDCommand;
import frc.robot.commands.Climber.ClimberUpCommand;
import frc.robot.commands.Climber.RunServoCommand;
import frc.robot.commands.Conveyor.ConveyorBackCommand;
import frc.robot.commands.Conveyor.ConveyorForwardCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorPIDCommand;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.Intake.GoUntilBeamBreakCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.IntakeGoCommand;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.LimeLight.ChangePipelineCommand;
import frc.robot.commands.Shooter.FlywheelSpinCommand;
import frc.robot.commands.Shooter.PivotCommand;
import frc.robot.commands.Shooter.PivotUpCommand;
import frc.robot.commands.Shooter.ShooterIntakeCommand;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  // To fix on Monday
  public ShooterPositions position;
  private final PivotSubsystem m_pivotSubsystem= new PivotSubsystem(position);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSub = new ClimberSubsystem();
  private final ElevatorSubsystem m_elevatorSub = new ElevatorSubsystem();
  private final Limelight m_Limelight = new Limelight();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(m_Limelight);
  private final ConveyorSubsystem m_conveyorSub = new ConveyorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(1);

  private final CommandXboxController driver = new CommandXboxController(1);
  private final Joystick controller = new Joystick(0);

  private final CommandXboxController driverController2 = 
      new CommandXboxController(2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, controller, 1, 0, 4, true, true));
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
    //ALL COMMANDS TEST: 
    driverController.y().onTrue(new ConveyorForwardCommand(m_conveyorSub));
    driverController.b().onTrue(new ConveyorBackCommand(m_conveyorSub));
    driverController.a().onTrue(new IntakeOutCommand(m_intakeSubsystem));
    driverController.x().onTrue(new IntakeGoCommand(m_intakeSubsystem));

    driverController.leftBumper().onTrue(new GoUntilBeamBreakCommand(m_intakeSubsystem));
    driverController.rightBumper().onTrue(new ElevatorDownCommand(m_elevatorSub));
    driverController.leftTrigger().onTrue(new ElevatorPIDCommand(m_elevatorSub));
    driverController.rightTrigger().onTrue(new ElevatorUpCommand(m_elevatorSub));

    driverController2.y().onTrue(new ClimberDownCommand(m_climberSub));
    driverController2.b().onTrue(new ClimberPIDCommand(m_climberSub));
    driverController2.a().onTrue(new ClimberUpCommand(m_climberSub));

    driverController2.x().onTrue(new RunServoCommand(m_climberSub, 0));
    driverController2.leftBumper().onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 0.8));
    driverController2.rightBumper().onTrue(new PivotCommand(m_pivotSubsystem, 0.8));
    driverController2.leftTrigger().onTrue(new PivotUpCommand(m_pivotSubsystem));

    // Flywheel Bindings
    //driverController.rightBumper().onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 1000)); // finished
    // Pivot Bindings
    //driverController.leftBumper().onTrue(new PivotCommand(m_pivotSubsystem, -100));

    //driverController.a().onTrue(new GoUntilBeamBreakCommand(m_intakeSubsystem));// finished :)
    //driverController.b().onTrue(new ClimberPIDCommand(m_climberSub));
    //driverController.x().onTrue(new ElevatorPIDCommand(m_elevatorSub));

    //driverController.pov(0).onTrue(new ConveyorForwardCommand(m_conveyorSub));//PLACEHOLDER BUTTONS
    //driverController.pov(180).onTrue(new ConveyorBackCommand(m_conveyorSub));
    //driverController.a().onTrue(new ElevatorDownCommand(m_elevatorSub));
    // Climber Bindings
    //driverController.x().onTrue(new ClimberDownCommand(null/*?????*/));
    //driverController.y().onTrue(new ClimberUpCommand(null/*?????*/));
    //driverController.leftBumper().whileTrue(new IntakeCommand(m_flywheelSubsystem, 0.3));
    //driverController.rightBumper().whileTrue(new IntakeCommand(m_flywheelSubsystem, -0.3));
    //driver.button(0, new FlywheelSpinCommand(null, driver));

    //driverController.a().onTrue(new ChangePipelineCommand(m_Limelight, 0));
    //driverController.b().onTrue(new ChangePipelineCommand(m_Limelight, 1));
    //driverController.y().onTrue(new ChangePipelineCommand(m_Limelight, 2));
    //driverController.x().onTrue(new ZeroGyroCommand(m_swerveSubsystem));

    //new JoystickButton(controller, 1).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
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
