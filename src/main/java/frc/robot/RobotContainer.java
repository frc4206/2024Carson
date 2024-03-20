// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Conveyor.ConveyorToPosition;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.commands.Intake.SetupNote;
import frc.robot.commands.LEDs.SetBlue;
import frc.robot.commands.LEDs.SetGreen;
import frc.robot.commands.LEDs.SetRed;
import frc.robot.commands.Intake.IntakeToDuty;
import frc.robot.commands.Pivot.PivotToDuty;
import frc.robot.commands.Pivot.PivotToPosition;
import frc.robot.commands.Pivot.ResetPivot;
import frc.robot.commands.Pivot.TogglePivotMode;
import frc.robot.commands.SYSTEMCHECK.SystemCheck;
import frc.robot.commands.Shooter.ShooterToVelocity;
import frc.robot.commands.Shooter.ToggleShooterToVelocity;
import frc.robot.commands.Shooter.ToggleShooterToVelocityIndividual;
import frc.robot.commands.Swerve.PID_to_game_Piece;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.ToggleAimed;
import frc.robot.commands.Swerve.ToggleAmped;
import frc.robot.commands.Swerve.ToggleFastRotate;
import frc.robot.commands.Swerve.TogglePickup;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.commands.combos.AutoShootAtSpeakerCommand;
import frc.robot.commands.combos.LineUpShotCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ClimberSubsystem m_leftClimberSubsystem = new ClimberSubsystem(
    Constants.Climber.climberLeftFollowID, true, 60, Constants.Climber.servoLeftID);
  private final ClimberSubsystem m_rightClimberSubsystem = new ClimberSubsystem(
    Constants.Climber.climberRightLeadID, false, 60, Constants.Climber.servoRightID);  
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final LEDs m_leds = new LEDs();
  public final Limelight m_Limelight = new Limelight();

  public static final int translationAxis = XboxController.Axis.kLeftY.value;
  public static final int strafeAxis = XboxController.Axis.kLeftX.value;
  public static final int rotationAxis = XboxController.Axis.kRightX.value;

  private final XboxController driva = new XboxController(Constants.OperatorConstants.drivaPort);
  private final XboxController operata = new XboxController(Constants.OperatorConstants.operataPort);
  private final XboxController operata2 = new XboxController(Constants.OperatorConstants.operata2Port);
  private final XboxController shootertesta = new XboxController(Constants.OperatorConstants.shootertestaPort);
  
  public RobotContainer() {
    NamedCommands.registerCommand("pivotSubwoofer", new PivotToPosition(m_pivotSubsystem, Constants.Pivot.subwooferPosition));
    NamedCommands.registerCommand("shoot", new ConveyorToDuty(m_conveyorSubsystem, 1).withTimeout(1.5));
    NamedCommands.registerCommand("intakeshort", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(0.6));
    NamedCommands.registerCommand("conveyorshort", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.4));
    NamedCommands.registerCommand("intake", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(0.75));
    NamedCommands.registerCommand("conveyor", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.5));
    NamedCommands.registerCommand("intakelong", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(1.5));
    NamedCommands.registerCommand("intakelonglong", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(1.9));
    NamedCommands.registerCommand("conveyorlong", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.6));
    NamedCommands.registerCommand("conveyorlonglong", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.8));
    NamedCommands.registerCommand("setupnote", new SetupNote(m_conveyorSubsystem, m_intakeSubsystem).withTimeout(0.8));
    NamedCommands.registerCommand("fly", new ShooterToVelocity(m_flywheelSubsystem, 6500));
    NamedCommands.registerCommand("Ai Pickup", new PID_to_game_Piece(m_swerveSubsystem, false, true, false, 2.5));//2.5
    NamedCommands.registerCommand("Ai Pickup long", new PID_to_game_Piece(m_swerveSubsystem, false, true, false, 3));//2.5
    NamedCommands.registerCommand("print", new InstantCommand(() -> System.out.println("Ran command in auto")));
    NamedCommands.registerCommand("startshot", new AutoShootAtSpeakerCommand(m_swerveSubsystem, m_flywheelSubsystem, m_pivotSubsystem).withTimeout(1));
    NamedCommands.registerCommand("lineupshot", new LineUpShotCommand(m_swerveSubsystem, m_flywheelSubsystem, m_pivotSubsystem));
    
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, driva, translationAxis, strafeAxis, rotationAxis, true, true));

    configureBindings();
  }

  private void setupClimberControls(XboxController controller) {
    m_leftClimberSubsystem.setupController(controller, XboxController.Axis.kLeftY.value);
    m_rightClimberSubsystem.setupController(controller, XboxController.Axis.kRightY.value);

    m_leftClimberSubsystem.engageServoPos = Constants.Climber.servo_left_engage;
    m_leftClimberSubsystem.disengageServoPos = Constants.Climber.servo_left_disengage;

    m_rightClimberSubsystem.engageServoPos = Constants.Climber.servo_right_engage;
    m_rightClimberSubsystem.disengageServoPos = Constants.Climber.servo_right_disengage;
  } 

	private boolean getLeftTrigger(XboxController controller) {
		return controller.getLeftTriggerAxis() > 0.05;
	}

  private boolean getRightTrigger(XboxController controller){
    return controller.getRightTriggerAxis() > 0.05;
  }
  
  private void configureBindings() {
    new JoystickButton(driva, 1).onTrue(new TogglePivotMode(m_pivotSubsystem));
    new JoystickButton(driva, 2).whileTrue(new ParallelCommandGroup(new IntakeToDuty(m_intakeSubsystem, -1), new ConveyorToDuty(m_conveyorSubsystem, -0.65)));
    new JoystickButton(driva, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
    new JoystickButton(driva, 4).onTrue(new ToggleAmped(m_swerveSubsystem));
    new JoystickButton(driva, 5).onTrue(new SetupNote(m_conveyorSubsystem, m_intakeSubsystem));
    new JoystickButton(driva, 6).whileTrue(new ParallelCommandGroup(new IntakeToDuty(m_intakeSubsystem, GlobalVariables.toAmpVelo ? 9/50 : 1), new ConveyorToDuty(m_conveyorSubsystem, 0.9)));
    new Trigger(() -> this.getLeftTrigger(driva)).onTrue(new ToggleShooterToVelocity(m_flywheelSubsystem, 6500));
    new Trigger(() -> this.getRightTrigger(driva)).onTrue(new ParallelCommandGroup(new InstantCommand(() -> m_pivotSubsystem.toggleAmpMode()), new ToggleShooterToVelocityIndividual(m_flywheelSubsystem, 3600, 2200)));
    new JoystickButton(driva, 7).onTrue(new TogglePickup(m_swerveSubsystem));
    new JoystickButton(driva, 8).onTrue(new ToggleAimed(m_swerveSubsystem));
    new POVButton(driva, 90).onTrue(new ToggleFastRotate());


    new JoystickButton(operata, 1).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.CLOSE)));
    new JoystickButton(operata, 2).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.PODIUM)));
    new JoystickButton(operata, 3).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER)));
    new JoystickButton(operata, 4).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.STAGE)));
    setupClimberControls(operata);
    

    new JoystickButton(operata2, 1).onTrue(new SystemCheck(m_leftClimberSubsystem, m_rightClimberSubsystem, m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem, operata2));
    new JoystickButton(operata2, 5).whileTrue(new SetBlue(m_leds));
    new JoystickButton(operata2, 6).whileTrue(new SetGreen(m_leds));
    new JoystickButton(operata2, 4).whileTrue(new SetRed(m_leds));


    
    new JoystickButton(shootertesta, 1).whileTrue(new PivotToPosition(m_pivotSubsystem, 1));
    new JoystickButton(shootertesta, 2).onTrue(new ShooterToVelocity(m_flywheelSubsystem, 4000));
    new JoystickButton(shootertesta, 4).whileTrue(new PivotToPosition(m_pivotSubsystem, 5));
    new JoystickButton(shootertesta, 5).onTrue(new ConveyorToPosition(m_conveyorSubsystem, 10));
    new JoystickButton(shootertesta, 6).onTrue(new ConveyorToPosition(m_conveyorSubsystem, 20));
    new JoystickButton(shootertesta, 7).whileTrue(new PivotToDuty(m_pivotSubsystem, -0.08));
    new JoystickButton(shootertesta, 8).whileTrue(new PivotToDuty(m_pivotSubsystem, 0.08));
    new JoystickButton(shootertesta, 9).onTrue(new ResetPivot(m_pivotSubsystem));
    new JoystickButton(shootertesta, 10).whileTrue(new IntakeToDuty(m_intakeSubsystem, 1));
    new POVButton(shootertesta, 90).onTrue(new AutoShootAtSpeakerCommand(m_swerveSubsystem, m_flywheelSubsystem, m_pivotSubsystem));
  }


  public Command getAutonomousCommand() {
    return new PathPlannerAuto("FirstShotTest");
  }
}