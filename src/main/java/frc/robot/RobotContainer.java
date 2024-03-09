// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.LimeLight.ChangePipelineCommand;
import frc.robot.autos.FourPieceLeftBlue;
import frc.robot.autos.FourPieceLeftRed;
import frc.robot.autos.FourPieceMiddleBlue;
import frc.robot.autos.FourPieceMiddleRed;
import frc.robot.autos.JustLeave;
import frc.robot.autos.JustLeaveRed;
import frc.robot.autos.SixPieceTakeoverBlue;
import frc.robot.autos.SixPieceTakeoverRed;
import frc.robot.autos.ThreePieceRightBlue;
import frc.robot.autos.ThreePieceRightRed;
import frc.robot.autos.TwoPieceMiddleBlue;
import frc.robot.autos.TwoPieceMiddleRed;
import frc.robot.commands.Climber.RunServosCommand;
import frc.robot.commands.Conveyor.ConveyerToSpeedCommand;
import frc.robot.commands.Elevator.ElevatorDownCommand;
import frc.robot.commands.Elevator.ElevatorPIDCommand;
import frc.robot.commands.Elevator.ElevatorUpCommand;
import frc.robot.commands.Elevator.PercentElevatorCommand;
import frc.robot.commands.Elevator.ResetElevatorCommand;
import frc.robot.commands.Intake.GoUntilNote;
import frc.robot.commands.Intake.IntakeToSpeedCommand;
import frc.robot.commands.Pivot.PercentPivotCommand;
import frc.robot.commands.Pivot.ResetPivotCommand;
import frc.robot.commands.Climber.ClimberDownCommand;
import frc.robot.commands.Climber.ClimberSetSpeedCommand;
import frc.robot.commands.Climber.ClimberStopCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimbDownLeftCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimbUpLeftCommand;
import frc.robot.commands.Climber.ClimberRight.ClimbDownRightCommand;
import frc.robot.commands.Climber.ClimberRight.ClimbUpRightCommand;
import frc.robot.commands.Climber.ClimberSetSpeedCommand.MotorDirections;
import frc.robot.commands.SYSTEMCHECK.SystemCheck;
import frc.robot.commands.Shooter.FlywheelSpinCommand;
import frc.robot.commands.Shooter.PercentShooterCommand;
import frc.robot.commands.Shooter.ShooterStopCommand;
import frc.robot.commands.Swerve.FreeHeadingState;
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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
	private final ConveyorSubsystem m_conveyorSub = new ConveyorSubsystem();
	private final ElevatorSubsystem m_elevatorSub = new ElevatorSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
	public final LEDSubsystem m_leds = new LEDSubsystem();
	public final Limelight m_Limelight = new Limelight();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    // private final ClimberSubsystem m_leftClimberSubsystem = new ClimberSubsystem(Constants.Climber.climberLeftFollowID, false, 40, Constants.Climber.servoLeftID);
    // private final ClimberSubsystem m_rightClimberSubsystem = new ClimberSubsystem(Constants.Climber.climberRightLeadID, false, 40, Constants.Climber.servoRightID);

	public static final int translationAxis = XboxController.Axis.kLeftY.value;
	public static final int strafeAxis = XboxController.Axis.kLeftX.value;
	public static final int rotationAxis = XboxController.Axis.kRightX.value;

	private final XboxController driver = new XboxController(5);
	private final XboxController operator = new XboxController(1);
	private final XboxController operator2 = new XboxController(2);
	private final XboxController shootertesta = new XboxController(3);
	private final XboxController elevatortesta = new XboxController(4);
	private final XboxController climbertesta = new XboxController(0);

	//private final ClimberSubsystem m_climberLeft = new ClimberSubsystem(Constants.Climber.climberLeftFollowID, false, Constants.Climber.servoLeftID, climbertesta.getPort());
	//private final ClimberSubsystem m_climberRight = new ClimberSubsystem(Constants.Climber.climberRightLeadID, true, Constants.Climber.servoRightID, climbertesta.getPort());
	
	final static SendableChooser<String> autoChooser = new SendableChooser<String>();

	public RobotContainer() {
			// m_flywheelSubsystem.setDefaultCommand(new PercentShooterCommand(m_flywheelSubsystem, 0.15));
		m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, true, true));
		//m_climberLeft.setDefaultCommand(new RunServosCommand(m_climberLeft, m_climberRight, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
		
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
		autoChooser.addOption("JustLeaveRed", "JustLeaveRed");
		autoChooser.addOption("JustLeaveBlue", "JustLeaveBlue");
		autoChooser.setDefaultOption("Nothing", "Nothing");

		SmartDashboard.putData(autoChooser);
		configureBindings();
	}

	private double getLeftStickY(XboxController controller){
		return -controller.getRawAxis(XboxController.Axis.kLeftY.value);
	}

	private boolean getLeftTrigger(XboxController controller) {
		return controller.getLeftTriggerAxis() > 0.02;
	}

	private boolean getRightTrigger(XboxController controller) {
		return controller.getRightTriggerAxis() > 0.02;
	}

	private boolean bumpersPressed(XboxController controller) {
		return controller.getLeftBumper() && controller.getRightBumper();
	}

	// private boolean backLeftButtonsPressed(XboxController controller) {
	// 	return controller.getAButton() && controller.getXButton();
	// }

	// private boolean backRightButtonsPressed(XboxController controller) {
	// 	return controller.getBButton() && controller.getYButton();
	// }
	
	private void configureBindings() {

		/* MAIN DRIVER CONTROLS!!! */
		// new JoystickButton(driver, 1).onTrue(new CyclePivotPositionCommand(m_pivotSubsystem));
		// new JoystickButton(driver, 1).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
		// new JoystickButton(driver, 1).onTrue(new ChangePipelineCommand(m_Limelight, 2));
		new JoystickButton(driver, 1).whileTrue(new PercentPivotCommand(m_pivotSubsystem, 0.02));
		// new JoystickButton(driver, 1).toggleOnTrue(Commands.startEnd(() -> m_pivotSubsystem.changePosition(ShooterPositions.NONE), () -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER), m_pivotSubsystem));
		// new JoystickButton(driver, 2).onTrue(new PivotCommand(m_pivotSubsystem));
		new JoystickButton(driver, 2).onTrue(new ResetPivotCommand(m_pivotSubsystem));
		// new JoystickButton(driver, 2).whileTrue(new ConveyerToSpeedCommand(m_conveyorSub, -0.8));
		new JoystickButton(driver, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
		// new JoystickButton(driver, 4).toggleOnTrue(Commands.startEnd(() -> m_climberSub.GoToSetpoint(Constants.Climber.climberTopSetpoint), () -> m_elevatorSub.GoToSetpoint(Constants.Climber.climberResetPosition), m_climberSub));
		// new JoystickButton(driver, 4).onTrue(new ResetPivotCommand(m_pivotSubsystem));
		new JoystickButton(driver, 4).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
		new JoystickButton(driver, 5).onTrue(new GoUntilNote(m_conveyorSub, m_intakeSubsystem));
		new JoystickButton(driver, 6).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, -0.5), new ConveyerToSpeedCommand(m_conveyorSub, 0.5)));
		// new Trigger(() -> this.getLeftTrigger(driver)).onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6500));
		// new Trigger(() -> this.getLeftTrigger(driver)).whileTrue(new PercentShooterCommand(m_flywheelSubsystem, 1));
		// new Trigger(() -> this.getRightTrigger(driver)).toggleOnTrue(Commands.startEnd(() -> m_elevatorSub.GoToSetpoint(Constants.Elevator.elevTrapPosition), () -> m_elevatorSub.GoToSetpoint(5), m_elevatorSub));
		// new Trigger(() -> this.getRightTrigger(driver)).toggleOnTrue(new ElevatorPIDCommand(m_elevatorSub, Constants.Elevator.elevTrapPosition));
		// new Trigger(() -> this.getRightTrigger(driver)).toggleOnFalse(new ElevatorPIDCommand(m_elevatorSub, 5));
		// new Trigger(() -> this.getRightTrigger(driver)).whileTrue(new ShooterStopCommand(m_flywheelSubsystem));
		// new JoystickButton(driver, 7).onTrue(new ChangePipelineCommand(m_Limelight, 2));
		new JoystickButton(driver, 7).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, 0.8), new ConveyerToSpeedCommand(m_conveyorSub, -0.675)));
		new JoystickButton(driver, 8).onTrue(new SetHeadingState(m_swerveSubsystem));



		/* OPERATOR CONTROLS!!! */
		new JoystickButton(operator, 1).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.CLOSE)));
		new JoystickButton(operator, 2).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.PODIUM)));
		new JoystickButton(operator, 3).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER)));
		new JoystickButton(operator, 4).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.STAGE)));
		new JoystickButton(operator2, 1).onTrue(new SystemCheck(m_climberSubsystem, m_conveyorSub, m_elevatorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem, operator2));

		

		/* SHOOTER CONTROLS!!! */
		new JoystickButton(shootertesta, 1).onTrue(new ShooterStopCommand(m_flywheelSubsystem));
		new JoystickButton(shootertesta, 4).whileTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6400));
		new JoystickButton(shootertesta, 6).whileTrue(new PercentShooterCommand(m_flywheelSubsystem, 1));



		/* ELEVATOR CONTROLS!!! */
		new JoystickButton(elevatortesta, 1).whileTrue(new PercentElevatorCommand(m_elevatorSub, -0.05));
		new JoystickButton(elevatortesta, 2).whileTrue(new ResetElevatorCommand(m_elevatorSub));
		new JoystickButton(elevatortesta, 4).whileTrue(new PercentElevatorCommand(m_elevatorSub, 0.05));
		new JoystickButton(elevatortesta, 5).whileTrue(new ElevatorDownCommand(m_elevatorSub));
		new JoystickButton(elevatortesta, 6).whileTrue(new ElevatorUpCommand(m_elevatorSub));
		new JoystickButton(elevatortesta, 7).whileTrue(new ElevatorPIDCommand(m_elevatorSub, 5));
		new JoystickButton(elevatortesta, 8).whileTrue(new ElevatorPIDCommand(m_elevatorSub, Constants.Elevator.elevTrapPosition+5));



		/* CLIMBER CONTROLS!!! */
		// new JoystickButton(climbertesta, 2).whileTrue(new RunServosCommand(m_climberLeft, m_climberRight, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
		// new JoystickButton(climbertesta, 3).whileTrue(new RunServosCommand(m_climberLeft, m_climberRight, Constants.Climber.servoPosLeftDisEngage, Constants.Climber.servoPosRightDisEngage));
		
		/* CLIMBER UP controls */
		// new Trigger(() -> this.getLeftTrigger(climbertesta)).whileTrue(new ClimbUpLeftCommand(m_climberLeft));
		// TODO: might need to remove this or just de-comment this //new Trigger(() -> this.getRightTrigger(climbertesta)).whileTrue(new ClimbUpRightCommand(m_climberSubsystem, climbertesta.getPort()));
		// TODO: might need to remove this or just de-comment this //new Trigger(() -> this.getLeftTrigger(climbertesta)).whileTrue(new ClimbUpLeftCommand(m_climberSubsystem, climbertesta.getPort()));
		// TODO: might need to remove this or just de-comment this //new Trigger(() -> this.getLeftTrigger(climbertesta) && this.getRightTrigger(climbertesta)).whileTrue(new ClimberSetSpeedCommand(m_climberSubsystem, MotorDirections.UP, climbertesta.getPort()));
		//new Trigger(() -> !this.getLeftTrigger(climbertesta) && !this.getRightTrigger(climbertesta)).onTrue(new ClimberSetSpeedCommand(m_climberSubsystem, MotorDirections.STOP, climbertesta.getPort()));

		if(climbertesta.getLeftTriggerAxis() >= 0.02 && climbertesta.getRightTriggerAxis() >= 0.02) {
			new ClimberSetSpeedCommand(m_climberSubsystem, MotorDirections.UP, climbertesta.getPort());
		} else if (climbertesta.getLeftTriggerAxis() >= 0.02) {
			new ClimbUpLeftCommand(m_climberSubsystem, climbertesta.getPort());
		} else if(climbertesta.getRightTriggerAxis() >= 0.02)/*the right trigger >= 0.02 */ {
			new ClimbUpLeftCommand(m_climberSubsystem, climbertesta.getPort());
		} else {
			new ClimberSetSpeedCommand(m_climberSubsystem, MotorDirections.STOP, climbertesta.getPort());
		}

		/* CLIMBER DOWN controls (individual) */
		new JoystickButton(climbertesta, 5).whileTrue(new ClimbDownLeftCommand(m_climberSubsystem));
		new JoystickButton(climbertesta, 6).whileTrue(new ClimbDownRightCommand(m_climberSubsystem));
		new Trigger(() -> this.bumpersPressed(climbertesta)).whileTrue(new ClimberSetSpeedCommand(m_climberSubsystem, MotorDirections.DOWN, climbertesta.getPort()));
		new Trigger(() -> !this.bumpersPressed(climbertesta)).onTrue(new ClimberSetSpeedCommand(m_climberSubsystem, MotorDirections.STOP, climbertesta.getPort()));

		/* SERVO controls */
		//new Trigger(() -> this.backLeftButtonsPressed(climbertesta)).onTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
		//new Trigger(() -> this.backRightButtonsPressed(climbertesta));

		// TODO: Add boolean for servoEngaged (both left and right) and create an if statement to detect (dis)engaged and set servo to opposite position.
	}

	
	public Command getAutonomousCommand() {
		String selectedAuto = autoChooser.getSelected();

		if (selectedAuto == "SixPieceRed"){
		return new SixPieceTakeoverRed(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		} 
		else if (selectedAuto == "SixPieceBlue"){
		return new SixPieceTakeoverBlue(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "FourPieceMiddleRed"){
		return new FourPieceMiddleRed(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "FourPieceMiddleBlue"){
		return new FourPieceMiddleBlue(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "TwoPieceMiddleRed"){
		return new TwoPieceMiddleRed(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "TwoPieceMiddleBlue"){
		return new TwoPieceMiddleBlue(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "FourPieceLeftRed"){
		return new FourPieceLeftRed(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "FourPieceLeftBlue"){
		return new FourPieceLeftBlue(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "ThreePieceRightRed"){
		return new ThreePieceRightRed(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "ThreePieceRightBlue"){
		return new ThreePieceRightBlue(m_conveyorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
		}
		else if (selectedAuto == "JustLeaveRed"){
		return new JustLeaveRed(m_swerveSubsystem, new Pose2d(new Translation2d(5, m_swerveSubsystem.getPoseInverted().getY()), new Rotation2d(0)));
		}
		else if (selectedAuto == "JustLeaveBlue"){
		return new JustLeave(m_swerveSubsystem, new Pose2d(new Translation2d(5, m_swerveSubsystem.getPose().getY()), new Rotation2d(0)));
		}
		else {
		return new InstantCommand();
		}
	}
}
