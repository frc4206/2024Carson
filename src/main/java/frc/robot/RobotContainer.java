// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.commands.Autos;
//import frc.robot.commands.Climber.VortexClimberDown;
//import frc.robot.commands.Climber.VortexClimberPIDCommand;
//import frc.robot.commands.Climber.VortexClimberUpCommand;
//import frc.robot.commands.Elevator.VortexElevatorDownCommand;
//import frc.robot.commands.Elevator.VortexElevatorPIDCommand;
//import frc.robot.commands.Intake.GoUntilBeamBreakCommand;
import frc.robot.commands.Intake.IntakeCommand;
//import frc.robot.commands.Intake.IntakeGo;
import frc.robot.commands.LimeLight.ChangePipelineCommand;
import frc.robot.commands.Shooter.ShooterToSpeaker;
import frc.robot.commands.Climber.ClimberDownLeftCommand;
import frc.robot.commands.Climber.ClimberDownRightCommand;
import frc.robot.commands.Climber.ClimberToggleUpCommand;
import frc.robot.commands.Climber.ClimberUpLeftCommand;
import frc.robot.commands.Climber.ClimberUpRightCommand;
import frc.robot.commands.Climber.RunServoLeftCommand;
import frc.robot.commands.Climber.RunServoRightCommand;
import frc.robot.commands.Conveyor.ConveyerToSpeedCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.Intake.GoUntilNote;
import frc.robot.commands.Intake.IntakeGoCommand;
import frc.robot.commands.Intake.IntakeToSpeedCommand;
import frc.robot.commands.LimeLight.ChangePipelineCommand;
import frc.robot.commands.Pivot.CyclePivotPositionCommand;
import frc.robot.commands.Pivot.PercentPivotCommand;
import frc.robot.commands.Pivot.PivotCommand;
import frc.robot.commands.Pivot.ResetPivotCommand;
import frc.robot.commands.Shooter.ShooterStopCommand;
import frc.robot.commands.Shooter.FlywheelSpinCommand;
import frc.robot.commands.Shooter.PercentShooterCommand;
import frc.robot.commands.Swerve.SetHeadingState;
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

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSub = new ClimberSubsystem();
  private final ElevatorSubsystem m_elevatorSub = new ElevatorSubsystem();
  private final Limelight m_Limelight = new Limelight();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ConveyorSubsystem m_conveyorSub = new ConveyorSubsystem();

  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(2);
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, driver, 1, 0, 4, true, true));
    configureBindings();
  }

  private boolean getLeftTrigger(XboxController controller){
    return controller.getLeftTriggerAxis() > 0.05;
  }

  private boolean getRightTrigger(XboxController controller){
    return controller.getRightTriggerAxis() > 0.05;
  }
  
  public void configureBindings() {
    // new JoystickButton(driver, 1).toggleOnTrue(new ShooterToSpeaker(m_flywheelSubsystem, m_pivotSubsystem));
    new JoystickButton(driver, 1).onTrue(new CyclePivotPositionCommand(m_pivotSubsystem));
    // new JoystickButton(driver, 1).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
    // new JoystickButton(driver, 1).onTrue(new ChangePipelineCommand(m_Limelight, 2));

    // new JoystickButton(driver, 2).whileTrue(new ConveyerToSpeedCommand(m_conveyorSub, -0.8));
    new JoystickButton(driver, 2).onTrue(new ResetPivotCommand(m_pivotSubsystem));
    // new JoystickButton(driver, 2).onTrue(new PivotCommand(m_pivotSubsystem));
    
    new JoystickButton(driver, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
    
    // new JoystickButton(driver, 4).toggleOnTrue(new ClimbToTopCommand(climber));
    // new JoystickButton(driver, 4).toggleOnFalse(new ClimbToBottomCommand(climber));
    new JoystickButton(operator, 1).whileTrue(new ClimberDownRightCommand(m_climberSub));
    new JoystickButton(operator, 2).whileTrue(new ClimberUpRightCommand(m_climberSub));
    new JoystickButton(operator, 6).whileTrue(new RunServoRightCommand(m_climberSub, 0.45));
    new Trigger(() -> this.getRightTrigger(operator)).whileTrue(new RunServoRightCommand(m_climberSub, 0.55));
    new JoystickButton(operator, 3).whileTrue(new ClimberDownLeftCommand(m_climberSub));
    new JoystickButton(operator, 4).whileTrue(new ClimberUpLeftCommand(m_climberSub));
    new JoystickButton(operator, 5).whileTrue(new RunServoLeftCommand(m_climberSub, 0.45));
    new Trigger(() -> this.getLeftTrigger(operator)).whileTrue(new RunServoLeftCommand(m_climberSub, 0.55));  
    
    //new JoystickButton(operator, 7).whileTrue(new )

    new JoystickButton(operator, 8).whileTrue(new ClimberToggleUpCommand(m_climberSub));//start


    // new JoystickButton(driver, 4).onTrue(new ResetPivotCommand(m_pivotSubsystem));
    //new JoystickButton(driver, 4).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
    
    // new Trigger(() -> this.getLeftTrigger(driver)).onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6500));
    new Trigger(() -> this.getLeftTrigger(driver)).onTrue(new PercentShooterCommand(m_flywheelSubsystem, 1));

    new Trigger(() -> this.getRightTrigger(driver)).onTrue(new ShooterStopCommand(m_flywheelSubsystem));
    // new Trigger(() -> this.getRightTrigger(driver)).toggleOnTrue(new CarriageToAmp());
    // new Trigger(() -> this.getRightTrigger(driver)).toggleOnTrue(new ElevatorToBottom());
    
    new JoystickButton(driver, 5).onTrue(new GoUntilNote(m_conveyorSub, m_intakeSubsystem));
    new JoystickButton(driver, 6).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, -1), new ConveyerToSpeedCommand(m_conveyorSub, 1)));

    // new JoystickButton(driver, 7).whileTrue(new ElevatorDownCommand(m_elevatorSub));
    // new JoystickButton(driver, 7).onTrue(new ChangePipelineCommand(m_Limelight, 2));
    // new JoystickButton(driver, 8).whileTrue(new ElevatorUpCommand(m_elevatorSub));
    new JoystickButton(driver, 8).onTrue(new SetHeadingState(m_swerveSubsystem));
  }

  
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Path");
    // return new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new GoUntilNote(m_conveyorSub, m_intakeSubsystem).until(() -> m_conveyorSub.hasNote()),
    //     new FlywheelSpinCommand(m_flywheelSubsystem, 6200).until(() -> m_flywheelSubsystem.shooterAtVelocity(6200))
    //   ),
    //   new ConveyerToSpeedCommand(m_conveyorSub, 1)
    // );
  }
}
