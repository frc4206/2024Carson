// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
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
import frc.robot.commands.Climber.ClimberAllMethodsCommand;
import frc.robot.commands.Climber.ClimberRunBothDownCommand;
import frc.robot.commands.Climber.ClimberRunBothUpCommand;
import frc.robot.commands.Climber.ClimberStopAllMotorsCommand;
import frc.robot.commands.Climber.ClimberToggleUpCommand;
import frc.robot.autos.FourPieceLeftBlue;
import frc.robot.autos.FourPieceLeftRed;
import frc.robot.autos.FourPieceMiddleBlue;
import frc.robot.autos.FourPieceMiddleRed;
import frc.robot.autos.JustLeaveBlue;
import frc.robot.autos.JustLeaveRed;
import frc.robot.autos.SixPieceTakeoverBlue;
import frc.robot.autos.SixPieceTakeoverRed;
import frc.robot.autos.ThreePieceRightBlue;
import frc.robot.autos.ThreePieceRightRed;
import frc.robot.autos.TwoPieceMiddleBlue;
import frc.robot.autos.TwoPieceMiddleRed;
import frc.robot.commands.Climber.ClimberDownCommand;
import frc.robot.commands.Climber.ClimberUpCommand;
import frc.robot.commands.Climber.RunServosCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimberDownLeftCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimberRunLeftCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimberUpLeftCommand;
import frc.robot.commands.Climber.ClimberRight.ClimberDownRightCommand;
import frc.robot.commands.Climber.ClimberRight.ClimberRunRightCommand;
import frc.robot.commands.Climber.ClimberRight.ClimberUpRightCommand;
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
=======
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.Climber.ClimberLeft.ClimbDownLeft;
import frc.robot.commands.Climber.ClimberLeft.ClimbUpLeft;
import frc.robot.commands.Climber.ClimberLeft.ServoLeftGoToPosition;
import frc.robot.commands.Climber.ClimberRight.ClimbDownRight;
import frc.robot.commands.Climber.ClimberRight.ClimbUpRight;
import frc.robot.commands.Climber.ClimberRight.ServoRightGoToPosition;
import frc.robot.commands.Conveyor.ConveyorToPosition;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.ElevatorToPosition;
import frc.robot.commands.Elevator.ElevatorUp;
import frc.robot.commands.Elevator.ElevatorToDuty;
import frc.robot.commands.Elevator.ResetElevator;
import frc.robot.commands.Elevator.ToggleElevatorAmp;
import frc.robot.commands.Intake.SetupNote;
import frc.robot.commands.Intake.IntakeToDuty;
import frc.robot.commands.Pivot.PivotToDuty;
import frc.robot.commands.Pivot.PivotToPosition;
import frc.robot.commands.Pivot.ResetPivot;
import frc.robot.commands.Pivot.TogglePivotMode;
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
import frc.robot.commands.SYSTEMCHECK.SystemCheck;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.Shooter.ShooterToVelocity;
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
import frc.robot.subsystems.ClimberLeftSubsystem;
import frc.robot.subsystems.ClimberRightSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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
<<<<<<< HEAD
	private final ConveyorSubsystem m_conveyorSub = new ConveyorSubsystem();
	private final ElevatorSubsystem m_elevatorSub = new ElevatorSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
	public final LEDSubsystem m_leds = new LEDSubsystem();
	public final Limelight m_Limelight = new Limelight();
=======
  private final ClimberSubsystem m_leftClimberSubsystem = new ClimberSubsystem(Constants.Climber.climberLeftFollowID, false, 40, Constants.Climber.servoLeftID);
  private final ClimberSubsystem m_rightClimberSubsystem = new ClimberSubsystem(Constants.Climber.climberRightLeadID, false, 40, Constants.Climber.servoRightID);
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public final LEDSubsystem m_leds = new LEDSubsystem();
  public final Limelight m_Limelight = new Limelight();
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362

	public static final int translationAxis = XboxController.Axis.kLeftY.value;
	public static final int strafeAxis = XboxController.Axis.kLeftX.value;
	public static final int rotationAxis = XboxController.Axis.kRightX.value;

<<<<<<< HEAD
	private final XboxController driver = new XboxController(5);
	private final XboxController operator = new XboxController(1);
	private final XboxController operator2 = new XboxController(2);
	private final XboxController shootertesta = new XboxController(3);
	private final XboxController elevatortesta = new XboxController(4);
	private final XboxController climbertesta = new XboxController(0);

	private final ClimberSubsystem m_climberSub = new ClimberSubsystem(climbertesta);
	private final ClimberLeftSubsystem m_climberLeftSub = new ClimberLeftSubsystem(climbertesta);
	private final ClimberRightSubsystem m_climberRightSub = new ClimberRightSubsystem(climbertesta);
	
	final static SendableChooser<String> autoChooser = new SendableChooser<String>();
=======
  private final XboxController driva = new XboxController(Constants.OperatorConstants.drivaPort);
  private final XboxController operata = new XboxController(Constants.OperatorConstants.operataPort);
  private final XboxController operata2 = new XboxController(Constants.OperatorConstants.operata2Port);
  private final XboxController shootertesta = new XboxController(Constants.OperatorConstants.shootertestaPort);
  private final XboxController elevatortesta = new XboxController(Constants.OperatorConstants.elevatortestaPort);
  private final XboxController climbertesta = new XboxController(Constants.OperatorConstants.climbertestaPort);
  
  final static SendableChooser<String> autoChooser = new SendableChooser<String>();

  public RobotContainer() {
    NamedCommands.registerCommand("shoot", new ConveyorToDuty(m_conveyorSubsystem, 1).withTimeout(1));
    NamedCommands.registerCommand("intakeshort", new IntakeToDuty(m_intakeSubsystem, -1).withTimeout(0.6));
    NamedCommands.registerCommand("conveyorshort", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.4));
    NamedCommands.registerCommand("intake", new IntakeToDuty(m_intakeSubsystem, -1).withTimeout(0.8));
    NamedCommands.registerCommand("conveyor", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.5));
    NamedCommands.registerCommand("intakelong", new IntakeToDuty(m_intakeSubsystem, -1).withTimeout(1));
    NamedCommands.registerCommand("conveyorlong", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.6));
    NamedCommands.registerCommand("pivot2", new PivotToPosition(m_pivotSubsystem, 4.65).withTimeout(0.75));
    NamedCommands.registerCommand("pivot3", new PivotToPosition(m_pivotSubsystem, 4.4).withTimeout(0.75));
    NamedCommands.registerCommand("pivot4", new PivotToPosition(m_pivotSubsystem, 2.92).withTimeout(0.75));
    
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
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362

	public RobotContainer() {
		// m_flywheelSubsystem.setDefaultCommand(new PercentShooterCommand(m_flywheelSubsystem, 0.15));
		m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, true, true));
		m_climberSub.setDefaultCommand(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
		
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

<<<<<<< HEAD
		SmartDashboard.putData(autoChooser);
		configureBindings();
	}

	private double getLeftStickY(XboxController controller){
		return -controller.getRawAxis(XboxController.Axis.kLeftY.value);
	}

=======
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
	private boolean getLeftTrigger(XboxController controller) {
		return controller.getLeftTriggerAxis() > 0.05;
	}

<<<<<<< HEAD
	private boolean getRightTrigger(XboxController controller) {
		return controller.getRightTriggerAxis() > 0.05;
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
		// new JoystickButton(driver, 1).onTrue(new CyclePivotPositionCommand(m_pivotSubsystem));
		// new JoystickButton(driver, 1).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
		// new JoystickButton(driver, 1).onTrue(new ChangePipelineCommand(m_Limelight, 2));
		new JoystickButton(driver, 1).whileTrue(new PercentPivotCommand(m_pivotSubsystem, 0.02));
		// new JoystickButton(driver, 1).toggleOnTrue(Commands.startEnd(() -> m_pivotSubsystem.changePosition(ShooterPositions.NONE), () -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER), m_pivotSubsystem));

		// new JoystickButton(driver, 2).onTrue(new PivotCommand(m_pivotSubsystem));
		new JoystickButton(driver, 2).onTrue(new ResetPivotCommand(m_pivotSubsystem));
		// new JoystickButton(driver, 2).whileTrue(new ConveyerToSpeedCommand(m_conveyorSub, -0.8));
		
		new JoystickButton(driver, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));

		new JoystickButton(operator, 2).whileTrue(new ClimberRunBothUpCommand(m_climberSub));
		new JoystickButton(operator, 3).whileTrue(new ClimberRunBothDownCommand(m_climberSub));
		new JoystickButton(operator, 5).whileTrue(new ClimberRunRightCommand(m_climberSub));
		new JoystickButton(operator, 1).whileTrue(new ClimberRunLeftCommand(m_climberSub)); 
		
		
		
		// new JoystickButton(driver, 4).toggleOnTrue(Commands.startEnd(() -> m_climberSub.GoToSetpoint(Constants.Climber.climberTopSetpoint), () -> m_elevatorSub.GoToSetpoint(Constants.Climber.climberResetPosition), m_climberSub));
		// new JoystickButton(driver, 4).onTrue(new ResetPivotCommand(m_pivotSubsystem));
		new JoystickButton(driver, 4).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
		
		new JoystickButton(driver, 5).onTrue(new GoUntilNote(m_conveyorSub, m_intakeSubsystem));
		new JoystickButton(driver, 6).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, -0.5), new ConveyerToSpeedCommand(m_conveyorSub, 0.5)));
		
		// new Trigger(() -> this.getLeftTrigger(driver)).onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6500));
		new Trigger(() -> this.getLeftTrigger(driver)).whileTrue(new PercentShooterCommand(m_flywheelSubsystem, 1));

		// new Trigger(() -> this.getRightTrigger(driver)).toggleOnTrue(Commands.startEnd(() -> m_elevatorSub.GoToSetpoint(Constants.Elevator.elevTrapPosition), () -> m_elevatorSub.GoToSetpoint(5), m_elevatorSub));
		// new Trigger(() -> this.getRightTrigger(driver)).toggleOnTrue(new ElevatorPIDCommand(m_elevatorSub, Constants.Elevator.elevTrapPosition));
		// new Trigger(() -> this.getRightTrigger(driver)).toggleOnFalse(new ElevatorPIDCommand(m_elevatorSub, 5));
		// new Trigger(() -> this.getRightTrigger(driver)).whileTrue(new ShooterStopCommand(m_flywheelSubsystem));

		// new JoystickButton(driver, 7).onTrue(new ChangePipelineCommand(m_Limelight, 2));
		new JoystickButton(driver, 7).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, 0.8), new ConveyerToSpeedCommand(m_conveyorSub, -0.675)));
		
		new JoystickButton(driver, 8).onTrue(new SetHeadingState(m_swerveSubsystem));


		new JoystickButton(operator, 1).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.CLOSE)));
		new JoystickButton(operator, 2).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.PODIUM)));
		new JoystickButton(operator, 3).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER)));
		new JoystickButton(operator, 4).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.STAGE)));
		new JoystickButton(operator, 5).whileTrue(new ClimberDownCommand(m_climberSub));
		new JoystickButton(operator, 6).whileTrue(new ClimberUpCommand(m_climberSub));

		//new Trigger(() -> this.getLeftTrigger(operator)).whileTrue(new ClimberUpLeftCommand(m_climberLeftSub));
		//new Trigger(() -> this.getRightTrigger(operator)).whileTrue(new ClimberUpRightCommand(m_climberRightSub));

		//new JoystickButton(operator, 7).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
		//new JoystickButton(operator, 8).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftDisEngage, Constants.Climber.servoPosRightDisEngage));
		

		new JoystickButton(operator2, 1).onTrue(new SystemCheck(m_climberSub, m_conveyorSub, m_elevatorSub, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem, operator2));



		
		new JoystickButton(shootertesta, 1).onTrue(new ShooterStopCommand(m_flywheelSubsystem));
		// new JoystickButton(shootertesta, 2).whileTrue(new InstantCommand());
		// new JoystickButton(shootertesta, 3).whileTrue(new InstantCommand());
		new JoystickButton(shootertesta, 4).whileTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6400));
		// new JoystickButton(shootertesta, 5).whileTrue(new InstantCommand());
		new JoystickButton(shootertesta, 6).whileTrue(new PercentShooterCommand(m_flywheelSubsystem, 1));


		new JoystickButton(elevatortesta, 1).whileTrue(new PercentElevatorCommand(m_elevatorSub, -0.05));
		new JoystickButton(elevatortesta, 2).whileTrue(new ResetElevatorCommand(m_elevatorSub));
		// new JoystickButton(elevatortesta, 3);
		new JoystickButton(elevatortesta, 4).whileTrue(new PercentElevatorCommand(m_elevatorSub, 0.05));
		new JoystickButton(elevatortesta, 5).whileTrue(new ElevatorDownCommand(m_elevatorSub));
		new JoystickButton(elevatortesta, 6).whileTrue(new ElevatorUpCommand(m_elevatorSub));
		new JoystickButton(elevatortesta, 7).whileTrue(new ElevatorPIDCommand(m_elevatorSub, 5));
		new JoystickButton(elevatortesta, 8).whileTrue(new ElevatorPIDCommand(m_elevatorSub, Constants.Elevator.elevTrapPosition+5));


		new JoystickButton(climbertesta, 1).whileTrue(new ClimberUpCommand(m_climberSub));

		new JoystickButton(climbertesta, 2).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
		new JoystickButton(climbertesta, 3).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftDisEngage, Constants.Climber.servoPosRightDisEngage));

		//new Trigger(() -> this.getLeftTrigger(climbertesta)).whileTrue(new ClimberUpLeftCommand(m_climberLeftSub));
		//new Trigger(() -> this.getRightTrigger(climbertesta)).whileTrue(new ClimberDownLeftCommand(m_climberLeftSub));
		new JoystickButton(climbertesta, 4).whileTrue(new ClimberUpCommand(m_climberSub));
		// new JoystickButton(climbertesta, 5).whileTrue(new ClimberDownLeftCommand(m_climberSub));
		// new JoystickButton(climbertesta, 6).whileTrue(new ClimberDownRightCommand(m_climberSub));
		
		/* CLIMBER UP controls (individual) */
		new Trigger(() -> this.getLeftTrigger(climbertesta)).whileTrue(new ClimberUpLeftCommand(m_climberSub));
		new Trigger(() -> this.getRightTrigger(climbertesta)).whileTrue(new ClimberUpRightCommand(m_climberSub));

		/* CLIMBER DOWN controls (individual) */
		new JoystickButton(climbertesta, 5).whileTrue(new ClimberDownLeftCommand(m_climberSub));
		new JoystickButton(climbertesta, 6).whileTrue(new ClimberDownRightCommand(m_climberSub));

		/* CLIMBER UP controls (together) */
		new Trigger(() -> this.getLeftTrigger(climbertesta) && this.getRightTrigger(climbertesta)).whileTrue(new ClimberRunBothUpCommand(m_climberSub));
		new Trigger(() -> !this.getLeftTrigger(climbertesta) && !this.getRightTrigger(climbertesta)).onTrue(new ClimberStopAllMotorsCommand(m_climberSub));

		/* CLIMBER DOWN controls (together) */
		new Trigger(() -> this.bumpersPressed(climbertesta)).whileTrue(new ClimberRunBothDownCommand(m_climberSub));
		new Trigger(() -> !this.bumpersPressed(climbertesta)).onTrue(new ClimberStopAllMotorsCommand(m_climberSub));

		/* SERVO controls */
		//new JoystickButton(climbertesta, 7).whileTrue(new InstantCommand());
		//new JoystickButton(climbertesta, 8).whileTrue(new InstantCommand());
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
		return new JustLeaveBlue(m_swerveSubsystem, new Pose2d(new Translation2d(5, m_swerveSubsystem.getPose().getY()), new Rotation2d(0)));
		}
		else {
		return new InstantCommand();
		}
	}
}
=======
  private boolean getRightTrigger(XboxController controller){
    return controller.getRightTriggerAxis() > 0.05;
  }
  
  private void configureBindings() {
    new JoystickButton(driva, 1).onTrue(new TogglePivotMode(m_pivotSubsystem));
    new JoystickButton(driva, 2).whileTrue(new ParallelCommandGroup(new IntakeToDuty(m_intakeSubsystem, 0.8), new ConveyorToDuty(m_conveyorSubsystem, -0.675)));
    new JoystickButton(driva, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
    // new JoystickButton(driva, 4).onTrue(new);
    new JoystickButton(driva, 5).onTrue(new SetupNote(m_conveyorSubsystem, m_intakeSubsystem));//.andThen(new shooterToDutyCommand(m_flywheelSubsystem, 1).onlyIf(() -> m_swerveSubsystem.shooterShouldRun())));
    new JoystickButton(driva, 6).whileTrue(new ParallelCommandGroup(new IntakeToDuty(m_intakeSubsystem, -0.8), new ConveyorToDuty(m_conveyorSubsystem, 0.5)));
    new Trigger(() -> this.getLeftTrigger(driva)).onTrue(new ShooterToVelocity(m_flywheelSubsystem, 6500));
    new Trigger(() -> this.getLeftTrigger(driva)).onFalse(new ShooterStop(m_flywheelSubsystem));
    new Trigger(() -> this.getRightTrigger(driva)).onTrue(new ToggleElevatorAmp(m_elevatorSubsystem));
    new JoystickButton(driva, 7).onTrue(new FreeHeadingState(m_swerveSubsystem));
    new JoystickButton(driva, 8).onTrue(new SetHeadingState(m_swerveSubsystem));

    // new Trigger(() -> this.getLeftTrigger(driva)).whileTrue(new shooterToDutyCommand2(m_flywheelSubsystem, 0.4, 0.6));
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
    new JoystickButton(operata, 7).whileTrue(new ClimbDown(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(operata, 8).whileTrue(new ClimbUp(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(operata, 5).whileTrue(new ClimbDownLeft(m_leftClimberSubsystem));
    new JoystickButton(operata, 6).whileTrue(new ClimbDownRight(m_rightClimberSubsystem));
    new Trigger(() -> this.getLeftTrigger(operata)).whileTrue(new ClimbUpLeft(m_leftClimberSubsystem));
    new Trigger(() -> this.getRightTrigger(operata)).whileTrue(new ClimbUpRight(m_rightClimberSubsystem));
    

    new JoystickButton(operata2, 1).onTrue(new SystemCheck(m_leftClimberSubsystem, m_rightClimberSubsystem, m_conveyorSubsystem, m_elevatorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem, operata2));



    new JoystickButton(shootertesta, 1).whileTrue(new PivotToPosition(m_pivotSubsystem, 1));
    new JoystickButton(shootertesta, 2).onTrue(new ShooterToVelocity(m_flywheelSubsystem, 6400));
    new JoystickButton(shootertesta, 3).onTrue(new ShooterStop(m_flywheelSubsystem));
    new JoystickButton(shootertesta, 4).whileTrue(new PivotToPosition(m_pivotSubsystem, 5));
    new JoystickButton(shootertesta, 5).onTrue(new ConveyorToPosition(m_conveyorSubsystem, 10));
    new JoystickButton(shootertesta, 6).onTrue(new ConveyorToPosition(m_conveyorSubsystem, 20));
    new JoystickButton(shootertesta, 7).whileTrue(new PivotToDuty(m_pivotSubsystem, -0.02));
    new JoystickButton(shootertesta, 8).whileTrue(new PivotToDuty(m_pivotSubsystem, 0.02));
    new JoystickButton(shootertesta, 9).onTrue(new ResetPivot(m_pivotSubsystem));


    new JoystickButton(elevatortesta, 1).whileTrue(new ElevatorToDuty(m_elevatorSubsystem, -0.1));
    new JoystickButton(elevatortesta, 2).whileTrue(new ResetElevator(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 4).whileTrue(new ElevatorToDuty(m_elevatorSubsystem, 0.1));
    new JoystickButton(elevatortesta, 5).whileTrue(new ElevatorDown(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 6).whileTrue(new ElevatorUp(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 7).whileTrue(new ElevatorToPosition(m_elevatorSubsystem, 5));
    new JoystickButton(elevatortesta, 8).whileTrue(new ElevatorToPosition(m_elevatorSubsystem, Constants.Elevator.elevatorTrapPosition+5));


    new JoystickButton(climbertesta, 7).whileTrue(new ClimbDown(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(climbertesta, 8).whileTrue(new ClimbUp(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(climbertesta, 5).whileTrue(new ClimbDownLeft(m_leftClimberSubsystem));
    new JoystickButton(climbertesta, 6).whileTrue(new ClimbDownRight(m_rightClimberSubsystem));
    new Trigger(() -> this.getLeftTrigger(climbertesta)).whileTrue(new ClimbUpLeft(m_leftClimberSubsystem));
    new Trigger(() -> this.getRightTrigger(climbertesta)).whileTrue(new ClimbUpRight(m_rightClimberSubsystem));
  }

  
  public Command getAutonomousCommand() {
    return new ParallelCommandGroup(new PathPlannerAuto("Amp1"), new ShooterToVelocity(m_flywheelSubsystem, 6500));
  }
} 
>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362
