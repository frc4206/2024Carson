// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autos.FourPieceLeftBlue;
import frc.robot.autos.FourPieceLeftRed;
import frc.robot.autos.FourPieceMiddleBlue;
import frc.robot.autos.FourPieceMiddleRed;
import frc.robot.autos.JustLeave;
import frc.robot.autos.SixPieceTakeoverBlue;
import frc.robot.autos.SixPieceTakeoverRed;
import frc.robot.autos.ThreePieceRightBlue;
import frc.robot.autos.ThreePieceRightRed;
import frc.robot.autos.TwoPieceMiddleBlue;
import frc.robot.autos.TwoPieceMiddleRed;
import frc.robot.commands.Climber.ClimbDownCommand;
import frc.robot.commands.Climber.ClimbUpCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimbDownLeftCommand;
import frc.robot.commands.Climber.ClimberLeft.ClimbUpLeftCommand;
import frc.robot.commands.Climber.ClimberLeft.ServoLeftGoToPosition;
import frc.robot.commands.Climber.ClimberRight.ClimbDownRightCommand;
import frc.robot.commands.Climber.ClimberRight.ClimbUpRightCommand;
import frc.robot.commands.Climber.ClimberRight.ServoRightGoToPosition;
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
import frc.robot.commands.SYSTEMCHECK.SystemCheck;
import frc.robot.commands.Shooter.ShooterStopCommand;
import frc.robot.commands.Shooter.FlywheelSpinCommand;
import frc.robot.commands.Shooter.PercentShooterCommand;
import frc.robot.commands.Swerve.SetHeadingState;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightEmittingDiodeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ClimberSubsystem m_leftClimberSubsystem = new ClimberSubsystem(Constants.Climber.climberLeftFollowID, true, 40, Constants.Climber.servoLeftID);
  private final ClimberSubsystem m_rightClimberSubsystem = new ClimberSubsystem(Constants.Climber.climberRightLeadID, false, 40, Constants.Climber.servoRightID);

  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  public final Limelight m_Limelight = new Limelight();
  public final LightEmittingDiodeSubsystem leds = new LightEmittingDiodeSubsystem(Constants.LED.pwm_port, Constants.LED.number_of_leds);
  public static final int translationAxis = XboxController.Axis.kLeftY.value;
  public static final int strafeAxis = XboxController.Axis.kLeftX.value;
  public static final int rotationAxis = XboxController.Axis.kRightX.value;

  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);
  private final XboxController operator2 = new XboxController(2);
  private final XboxController shootertesta = new XboxController(3);
  private final XboxController elevatortesta = new XboxController(4);
  private final XboxController climbertesta = new XboxController(5);
  
  final static SendableChooser<String> autoChooser = new SendableChooser<String>();

  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, true, true));
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

  private void setupClimberControls()
  {
    if(!m_leftClimberSubsystem.setupController(climbertesta, XboxController.Axis.kLeftY.value) || !m_rightClimberSubsystem.setupController(climbertesta, XboxController.Axis.kRightY.value)){
      System.err.println("Coult not bind climber controller to the subsystem.");
      System.exit(-1);
    };

    m_leftClimberSubsystem.engageServoPos = Constants.Climber.servoPosLeftEngage;
    m_leftClimberSubsystem.disengageServoPos = Constants.Climber.servoPosLeftDisEngage;

    m_rightClimberSubsystem.engageServoPos = Constants.Climber.servoPosRightEngage;
    m_rightClimberSubsystem.disengageServoPos = Constants.Climber.servoPosRightDisEngage;
  }
  
  private void configureBindings() {
    // new JoystickButton(driver, 1).onTrue(new CyclePivotPositionCommand(m_pivotSubsystem));
    // new JoystickButton(driver, 1).onTrue(new ChangePipelineCommand(m_Limelight, 2));
    new JoystickButton(driver, 1).whileTrue(new PercentPivotCommand(m_pivotSubsystem, -0.02));
    // new JoystickButton(driver, 1).toggleOnTrue(Commands.startEnd(() -> m_pivotSubsystem.changePosition(ShooterPositions.NONE), () -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER), m_pivotSubsystem));

    // new JoystickButton(driver, 2).onTrue(new PivotCommand(m_pivotSubsystem));
    new JoystickButton(driver, 2).onTrue(new ResetPivotCommand(m_pivotSubsystem));
    // new JoystickButton(driver, 2).whileTrue(new ConveyerToSpeedCommand(m_conveyorSubsystem, -0.8));
    
    new JoystickButton(driver, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
    
    // new JoystickButton(driver, 4).whileTrue(new PercentPivotCommand(m_pivotSubsystem, 0.02));
    // new JoystickButton(driver, 4).onTrue(new ElevatorPIDCommand(m_elevatorSubsystem, 5));
    new JoystickButton(driver, 4).whileTrue(new ElevatorDownCommand(m_elevatorSubsystem));
     
    new JoystickButton(driver, 5).onTrue(new GoUntilNote(m_conveyorSubsystem, m_intakeSubsystem));
    new JoystickButton(driver, 6).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, -0.5), new ConveyerToSpeedCommand(m_conveyorSubsystem, 0.5)));
    
    // new Trigger(() -> this.getLeftTrigger(driver)).onTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6500));
    new Trigger(() -> this.getLeftTrigger(driver)).whileTrue(new PercentShooterCommand(m_flywheelSubsystem, 1));

    // new Trigger(() -> this.getRightTrigger(driver)).onTrue(new ElevatorPIDCommand(m_elevatorSubsystem, Constants.Elevator.elevTrapPosition));
    new Trigger(() -> this.getRightTrigger(driver)).whileTrue(new ElevatorUpCommand(m_elevatorSubsystem));

    // new JoystickButton(driver, 7).onTrue(new ChangePipelineCommand(m_Limelight, 2));
    new JoystickButton(driver, 7).whileTrue(new ParallelCommandGroup(new IntakeToSpeedCommand(m_intakeSubsystem, 0.8), new ConveyerToSpeedCommand(m_conveyorSubsystem, -0.675)));
    
    new JoystickButton(driver, 8).onTrue(new SetHeadingState(m_swerveSubsystem));


    new JoystickButton(operator, 1).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.CLOSE)));
    new JoystickButton(operator, 2).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.PODIUM)));
    new JoystickButton(operator, 3).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER)));
    new JoystickButton(operator, 4).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.STAGE)));
    new JoystickButton(operator, 7).whileTrue(new ClimbDownCommand(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(operator, 8).whileTrue(new ClimbUpCommand(m_leftClimberSubsystem, m_rightClimberSubsystem));
    new JoystickButton(operator, 5).whileTrue(new ClimbDownLeftCommand(m_leftClimberSubsystem));
    new JoystickButton(operator, 6).whileTrue(new ClimbDownRightCommand(m_rightClimberSubsystem));
    new Trigger(() -> this.getLeftTrigger(operator)).whileTrue(new ClimbUpLeftCommand(m_leftClimberSubsystem));
    new Trigger(() -> this.getRightTrigger(operator)).whileTrue(new ClimbUpRightCommand(m_rightClimberSubsystem));
    

    new JoystickButton(operator2, 1).onTrue(new SystemCheck(m_leftClimberSubsystem, m_rightClimberSubsystem, m_conveyorSubsystem, m_elevatorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem, operator2));



    
    new JoystickButton(shootertesta, 1).onTrue(new ShooterStopCommand(m_flywheelSubsystem));
    // new JoystickButton(shootertesta, 2).whileTrue(new InstantCommand());
    // new JoystickButton(shootertesta, 3).whileTrue(new InstantCommand());
    new JoystickButton(shootertesta, 4).whileTrue(new FlywheelSpinCommand(m_flywheelSubsystem, 6400));
    // new JoystickButton(shootertesta, 5).whileTrue(new InstantCommand());
    new JoystickButton(shootertesta, 6).whileTrue(new PercentShooterCommand(m_flywheelSubsystem, 1));


    new JoystickButton(elevatortesta, 1).whileTrue(new PercentElevatorCommand(m_elevatorSubsystem, -0.05));
    new JoystickButton(elevatortesta, 2).whileTrue(new ResetElevatorCommand(m_elevatorSubsystem));
    // new JoystickButton(elevatortesta, 3);
    new JoystickButton(elevatortesta, 4).whileTrue(new PercentElevatorCommand(m_elevatorSubsystem, 0.05));
    new JoystickButton(elevatortesta, 5).whileTrue(new ElevatorDownCommand(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 6).whileTrue(new ElevatorUpCommand(m_elevatorSubsystem));
    new JoystickButton(elevatortesta, 7).whileTrue(new ElevatorPIDCommand(m_elevatorSubsystem, 5));
    new JoystickButton(elevatortesta, 8).whileTrue(new ElevatorPIDCommand(m_elevatorSubsystem, Constants.Elevator.elevTrapPosition+5));


    setupClimberControls();
  }

  
  public Command getAutonomousCommand() {
    String selectedAuto = autoChooser.getSelected();

    if (selectedAuto == "SixPieceRed"){
      return new SixPieceTakeoverRed(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    } 
    else if (selectedAuto == "SixPieceBlue"){
      return new SixPieceTakeoverBlue(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "FourPieceMiddleRed"){
      return new FourPieceMiddleRed(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "FourPieceMiddleBlue"){
      return new FourPieceMiddleBlue(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "TwoPieceMiddleRed"){
      return new TwoPieceMiddleRed(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "TwoPieceMiddleBlue"){
      return new TwoPieceMiddleBlue(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "FourPieceLeftRed"){
      return new FourPieceLeftRed(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "FourPieceLeftBlue"){
      return new FourPieceLeftBlue(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "ThreePieceRightRed"){
      return new ThreePieceRightRed(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "ThreePieceRightBlue"){
      return new ThreePieceRightBlue(m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem);
    }
    else if (selectedAuto == "JustLeave"){
      if (GlobalVariables.alliance == Alliance.Red){
        return new JustLeave(m_swerveSubsystem, new Pose2d(new Translation2d(5, m_swerveSubsystem.getPoseInverted().getY()), new Rotation2d(0)));
      } else {
        return new JustLeave(m_swerveSubsystem, new Pose2d(new Translation2d(5, m_swerveSubsystem.getPose().getY()), new Rotation2d(0)));
      }
    }
    else {
      return new InstantCommand();
    }
  }
}