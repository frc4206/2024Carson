// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Climber.ClimbDownCommand;
import frc.robot.commands.Climber.ClimbUpCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimbDownLeftCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimbUpLeftCommand;
import frc.robot.commands.Climber.ClimberLeft.ServoLeftGoToPosition;
import frc.robot.commands.Climber.ClimberRight.ClimbDownRightCommand;
import frc.robot.commands.Climber.ClimberRight.ClimbUpRightCommand;
import frc.robot.commands.Climber.ClimberRight.ServoRightGoToPosition;
import frc.robot.commands.Conveyor.ConveyerToPosition;
import frc.robot.commands.Conveyor.ConveyerToSpeedCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorPIDCommand;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.Elevator.PercentElevatorCommand;
import frc.robot.commands.Elevator.ResetElevatorCommand;
import frc.robot.commands.Elevator.ToggleElevatorAmp;
import frc.robot.commands.Intake.GoUntilNote;
import frc.robot.commands.Intake.IntakeToSpeedCommand;
import frc.robot.commands.Pivot.PercentPivotCommand;
import frc.robot.commands.Pivot.PivotCommand;
import frc.robot.commands.Pivot.ResetPivotCommand;
import frc.robot.commands.Pivot.TogglePivotMode;
import frc.robot.commands.SYSTEMCHECK.SystemCheck;
import frc.robot.commands.Shooter.ShooterStopCommand;
import frc.robot.commands.Shooter.FlywheelSpinCommand;
import frc.robot.commands.Swerve.SetHeadingState;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;
import frc.robot.subsystems.ClimbLeftSubsystem;
import frc.robot.subsystems.ClimbRightSubystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ClimbLeftSubsystem m_leftClimberSubsystem = new ClimbLeftSubsystem();
  private final ClimbRightSubystem m_rightClimberSubsystem = new ClimbRightSubystem();
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public final LEDSubsystem m_leds = new LEDSubsystem();
  public final Limelight m_Limelight = new Limelight();

  public static final int translationAxis = XboxController.Axis.kLeftY.value;
  public static final int strafeAxis = XboxController.Axis.kLeftX.value;
  public static final int rotationAxis = XboxController.Axis.kRightX.value;

  private final XboxController driva = new XboxController(Constants.OperatorConstants.drivaPort);
  private final XboxController operata = new XboxController(Constants.OperatorConstants.operataPort);
  private final XboxController operata2 = new XboxController(Constants.OperatorConstants.operata2Port);
  private final XboxController shootertesta = new XboxController(Constants.OperatorConstants.shootertestaPort);
  private final XboxController elevatortesta = new XboxController(Constants.OperatorConstants.elevatortestaPort);
  private final XboxController climbertesta = new XboxController(Constants.OperatorConstants.climbertestaPort);
  
  final static SendableChooser<String> autoChooser = new SendableChooser<String>();

  public RobotContainer() {
    NamedCommands.registerCommand("pivot2", new PivotCommand(m_pivotSubsystem, 4.65).withTimeout(0.75));
    NamedCommands.registerCommand("shoot", new ConveyerToSpeedCommand(m_conveyorSubsystem, 1).withTimeout(1));
    NamedCommands.registerCommand("intake", new IntakeToSpeedCommand(m_intakeSubsystem, -1).withTimeout(0.8));
    NamedCommands.registerCommand("conveyor", new ConveyerToSpeedCommand(m_conveyorSubsystem, 0.8).withTimeout(0.5));
    NamedCommands.registerCommand("pivot3", new PivotCommand(m_pivotSubsystem, 4.4).withTimeout(0.75));
    
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, driva, translationAxis, strafeAxis, rotationAxis, true, true));
    m_leftClimberSubsystem.setDefaultCommand(new ServoLeftGoToPosition(m_leftClimberSubsystem, Constants.Climber.servoPosLeftEngage));
    m_rightClimberSubsystem.setDefaultCommand(new ServoRightGoToPosition(m_rightClimberSubsystem, Constants.Climber.servoPosRightEngage));

    autoChooser.addOption("FourPieceLeftRed", "FourPieceLeftRed");
    autoChooser.addOption("FourPieceLeftBlue", "FourPieceLeftBlue");
    autoChooser.addOption("FourPieceMiddleRed", "FourPieceMiddleRed");
    autoChooser.addOption("FourPieceMiddleBlue", "FourPieceMiddleBlue");
    autoChooser.addOption("TwoPieceMiddleRed", "TwoPieceMiddleRed");
    autoChooser.addOption("TwoPieceMiddleBlue", "TwoPieceMiddleBlue");
    autoChooser.addOption("ThreePieceRightRed", "ThreePieceRightRed");
    autoChooser.addOption("ThreePieceRightBlue", "ThreePieceRightBlue");
    autoChooser.addOption("SixPieceRed", "SixPieceRed");
    autoChooser.addOption("SixPieceBlue", "SixPieceBlue");
    autoChooser.addOption("JustLeave", "JustLeave");
    autoChooser.setDefaultOption("Nothing", "Nothing");

    SmartDashboard.putData(autoChooser);
    configureBindings();
  }

	private boolean getLeftTrigger(XboxController controller) {
		return controller.getLeftTriggerAxis() > 0.05;
	}

  private boolean getRightTrigger(XboxController controller){
    return controller.getRightTriggerAxis() > 0.05;
  }
  
  private void configureBindings() {
    new JoystickButton(driva, 1).onTrue(new TogglePivotMode(m_pivotSubsystem));
    new JoystickButton(driva, 2).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, 0.8), new ConveyerToSpeedCommand(m_conveyorSubsystem, -0.675)));
    new JoystickButton(driva, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
    // new JoystickButton(driva, 4).onTrue(new);
    new JoystickButton(driva, 5).onTrue(new GoUntilNote(m_conveyorSubsystem, m_intakeSubsystem));//.andThen(new PercentShooterCommand(m_flywheelSubsystem, 1).onlyIf(() -> m_swerveSubsystem.shooterShouldRun())));
    new JoystickButton(driva, 6).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, -0.8), new ConveyerToSpeedCommand(m_conveyorSubsystem, 0.5)));
    new Trigger(() -> this.getLeftTrigger(driva)).onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6000));
    new Trigger(() -> this.getRightTrigger(driva)).onTrue(new ToggleElevatorAmp(m_elevatorSubsystem));
    new JoystickButton(driva, 8).onTrue(new SetHeadingState(m_swerveSubsystem));

    // new Trigger(() -> this.getLeftTrigger(driva)).whileTrue(new PercentShooterCommand2(m_flywheelSubsystem, 0.4, 0.6));
    /*
     * A: toggle shooter mode
     * X: zero gyro
     * Y: climb toggle
     * B: outtake
     * LB: intake
     * RB: shoot
     * LT: flyup toggle
     * RT: amp toggle
     * start: set heading state
    */


    new JoystickButton(operata, 1).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.CLOSE)));
    new JoystickButton(operata, 2).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.PODIUM)));
    new JoystickButton(operata, 3).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER)));
    new JoystickButton(operata, 4).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.STAGE)));
    new JoystickButton(operata, 7).whileTrue(new ClimbDownCommand(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(operata, 8).whileTrue(new ClimbUpCommand(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(operata, 5).whileTrue(new ClimbDownLeftCommand(m_leftClimberSubsystem));
    new JoystickButton(operata, 6).whileTrue(new ClimbDownRightCommand(m_rightClimberSubsystem));
    new Trigger(() -> this.getLeftTrigger(operata)).whileTrue(new ClimbUpLeftCommand(m_leftClimberSubsystem));
    new Trigger(() -> this.getRightTrigger(operata)).whileTrue(new ClimbUpRightCommand(m_rightClimberSubsystem));
    

    new JoystickButton(operata2, 1).onTrue(new SystemCheck(m_leftClimberSubsystem, m_rightClimberSubsystem, m_conveyorSubsystem, m_elevatorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem, operata2));



    new JoystickButton(shootertesta, 1).whileTrue(new PivotCommand(m_pivotSubsystem, 1));
    new JoystickButton(shootertesta, 2).onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6400));
    new JoystickButton(shootertesta, 3).onTrue(new ShooterStopCommand(m_flywheelSubsystem));
    new JoystickButton(shootertesta, 4).whileTrue(new PivotCommand(m_pivotSubsystem, 5));
    new JoystickButton(shootertesta, 5).onTrue(new ConveyerToPosition(m_conveyorSubsystem, 10));
    new JoystickButton(shootertesta, 6).onTrue(new ConveyerToPosition(m_conveyorSubsystem, 20));
    new JoystickButton(shootertesta, 7).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
    new JoystickButton(shootertesta, 8).whileTrue(new PercentPivotCommand(m_pivotSubsystem, 0.02));
    new JoystickButton(shootertesta, 9).onTrue(new ResetPivotCommand(m_pivotSubsystem));


    new JoystickButton(elevatortesta, 1).whileTrue(new PercentElevatorCommand(m_elevatorSubsystem, -0.1));
    new JoystickButton(elevatortesta, 2).whileTrue(new ResetElevatorCommand(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 4).whileTrue(new PercentElevatorCommand(m_elevatorSubsystem, 0.1));
    new JoystickButton(elevatortesta, 5).whileTrue(new ElevatorDownCommand(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 6).whileTrue(new ElevatorUpCommand(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 7).whileTrue(new ElevatorPIDCommand(m_elevatorSubsystem, 5));
    new JoystickButton(elevatortesta, 8).whileTrue(new ElevatorPIDCommand(m_elevatorSubsystem, Constants.Elevator.elevTrapPosition+5));


    new JoystickButton(climbertesta, 7).whileTrue(new ClimbDownCommand(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(climbertesta, 8).whileTrue(new ClimbUpCommand(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(climbertesta, 5).whileTrue(new ClimbDownLeftCommand(m_leftClimberSubsystem));
    new JoystickButton(climbertesta, 6).whileTrue(new ClimbDownRightCommand(m_rightClimberSubsystem));
    new Trigger(() -> this.getLeftTrigger(climbertesta)).whileTrue(new ClimbUpLeftCommand(m_leftClimberSubsystem));
    new Trigger(() -> this.getRightTrigger(climbertesta)).whileTrue(new ClimbUpRightCommand(m_rightClimberSubsystem));
  }

  
  public Command getAutonomousCommand() {
    return new ParallelCommandGroup(new PathPlannerAuto("Source1"), new FlywheelSpinCommand(m_flywheelSubsystem, 6500));
  }
}