// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autos.FourPieceLeftBlue;
import frc.robot.autos.FourPieceLeftRed;
import frc.robot.autos.FourPieceMiddleBlue;
import frc.robot.autos.FourPieceMiddleRed;
import frc.robot.autos.JustLeaveRed;
import frc.robot.autos.SixPieceTakeoverBlue;
import frc.robot.autos.SixPieceTakeoverRed;
import frc.robot.autos.ThreePieceRightBlue;
import frc.robot.autos.ThreePieceRightRed;
import frc.robot.autos.TwoPieceMiddleBlue;
import frc.robot.autos.TwoPieceMiddleRed;
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
import frc.robot.commands.Swerve.PID_DistanceOdometry2;
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
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
    // m_flywheelSubsystem.setDefaultCommand(new PercentShooterCommand(m_flywheelSubsystem, 0.15));
    m_swerveSubsystem.setDefaultCommand(new TeleopSwerve(m_swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, true, true));
   
    
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
		return controller.getLeftTriggerAxis() > 0.05;
	}

  private boolean getRightTrigger(XboxController controller){
    return controller.getRightTriggerAxis() > 0.05;
  }
  
  private void configureBindings() {
    new JoystickButton(driver, 3).onTrue(new ZeroGyroCommand(m_swerveSubsystem));
    
    new JoystickButton(driver, 1).whileTrue(new PID_DistanceOdometry2(m_swerveSubsystem, true,  true,13, 6, 180, 10, false));
    new JoystickButton(operator, 1).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.CLOSE)));
    new JoystickButton(operator, 2).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.PODIUM)));
    new JoystickButton(operator, 3).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.UNDER)));
    new JoystickButton(operator, 4).onTrue(new InstantCommand(() -> m_pivotSubsystem.changePosition(ShooterPositions.STAGE)));}
    /* 
    new J·oystickButton(operator, 5).whileTrue(new ClimberDownCommand(m_climberSub));
    new JoystickButton(operator, 6).whileTrue(new ClimberUpCommand(m_climberSub));
    new Trigger(() -> this.getLeftTrigger(operator)).whileTrue(new ClimberUpLeftCommand(m_climberSub));
    new Trigger(() -> this.getRightTrigger(operator)).whileTrue(new ClimberUpRightCommand(m_climberSub));

    //new JoystickButton(operator, 7).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
    //new JoystickButton(operator, 8).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftDisEngage, Constants.Climber.servoPosRightDisEngage));
    new JoystickButton(operator, 7).whileTrue(new ClimberDownLeftCommand(m_climberSub));
    new JoystickButton(operator, 8).whileTrue(new ClimberDownRightCommand(m_climberSub));
    

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


    new JoystickButton(climbertesta, 1).whileTrue(new ClimberDownCommand(m_climberSub));

    new JoystickButton(climbertesta, 2).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftEngage, Constants.Climber.servoPosRightEngage));
    new JoystickButton(climbertesta, 3).whileTrue(new RunServosCommand(m_climberSub, Constants.Climber.servoPosLeftDisEngage, Constants.Climber.servoPosRightDisEngage));

    new JoystickButton(climbertesta, 4).whileTrue(new ClimberUpCommand(m_climberSub));
    new JoystickButton(climbertesta, 5).whileTrue(new ClimberDownLeftCommand(m_climberSub));
    new JoystickButton(climbertesta, 6).whileTrue(new ClimberDownRightCommand(m_climberSub));
    new Trigger(() -> this.getLeftTrigger(climbertesta)).whileTrue(new ClimberUpLeftCommand(m_climberSub));
    new Trigger(() -> this.getRightTrigger(climbertesta)).whileTrue(new ClimberUpRightCommand(m_climberSub));
    new JoystickButton(climbertesta, 7).whileTrue(new InstantCommand());
    new JoystickButton(climbertesta, 8).whileTrue(new InstantCommand());
  }*/

  
  public Command getAutonomousCommand() {
    String selectedAuto = autoChooser.getSelected();
/* 
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
    }*/
    return null;
  }
}