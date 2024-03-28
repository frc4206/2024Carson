// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Conveyor.ConveyorToPosition;
import frc.lib.util.TunerConstants;
import frc.robot.COMBOS.Outtake;
import frc.robot.COMBOS.Shoot;
import frc.robot.COMBOS.ShooterToAmp;
import frc.robot.COMBOS.ShooterToSpeaker;
import frc.robot.commands.AmpBar.AmpBarToDuty;
import frc.robot.commands.AmpBar.AmpBarToPosition;
import frc.robot.commands.Conveyor.ConveyorToDuty;
import frc.robot.commands.Intake.SetupNote;
import frc.robot.commands.Intake.IntakeToDuty;
import frc.robot.commands.Pivot.ChangePivotPosition;
import frc.robot.commands.Pivot.PivotToDuty;
import frc.robot.commands.Pivot.PivotToPosition;
import frc.robot.commands.Pivot.ResetPivot;
import frc.robot.commands.Pivot.ToggleAutoMode;
import frc.robot.commands.SYSTEMCHECK.SystemCheck;
import frc.robot.commands.Shooter.ShooterToDuty;
import frc.robot.commands.Shooter.ShooterToVelocity;
import frc.robot.commands.Swerve.PID_to_game_Piece;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Swerve.ToggleAimed;
import frc.robot.commands.Swerve.ToggleAmped;
import frc.robot.commands.Swerve.ToggleFastRotate;
import frc.robot.commands.Swerve.TogglePickup;
import frc.robot.commands.Swerve.ZeroGyroCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;
import frc.robot.subsystems.AmpBarSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public final AmpBarSubsystem m_ampBarSubsystem = new AmpBarSubsystem();
  public final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  public final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  public final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final SwerveSubsystem m_swerveSubsystem = TunerConstants.DriveTrain;
  public final LEDs m_leds = new LEDs();
  public final Limelight m_Limelight = new Limelight();

  private static final int translationAxis = XboxController.Axis.kLeftY.value;
  private static final int strafeAxis = XboxController.Axis.kLeftX.value;
  private static final int rotationAxis = XboxController.Axis.kRightX.value;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final XboxController driva = new XboxController(Constants.OperatorConstants.drivaPort);
  private final XboxController operata = new XboxController(Constants.OperatorConstants.operataPort);
  private final XboxController operata2 = new XboxController(Constants.OperatorConstants.operata2Port);
  private final XboxController shootertesta = new XboxController(Constants.OperatorConstants.shootertestaPort);
  private final XboxController ampBartesta = new XboxController(Constants.OperatorConstants.ampBartestaPort);
  
  public RobotContainer() {
    NamedCommands.registerCommand("pivotsub", new PivotToPosition(m_pivotSubsystem, Constants.Pivot.subwooferPosition).withTimeout(0.25));
    NamedCommands.registerCommand("pivotamp1", new PivotToPosition(m_pivotSubsystem, 4.25).withTimeout(0.25));
    NamedCommands.registerCommand("pivotamp2", new PivotToPosition(m_pivotSubsystem, 4.5).withTimeout(0.25));
    NamedCommands.registerCommand("pivotcleanup1", new PivotToPosition(m_pivotSubsystem, 8).withTimeout(0.25));
    NamedCommands.registerCommand("pivotsource1", new PivotToPosition(m_pivotSubsystem, 7).withTimeout(0.25));
    NamedCommands.registerCommand("pivotsource2", new PivotToPosition(m_pivotSubsystem, 8).withTimeout(0.25));
    NamedCommands.registerCommand("pivotsource3", new PivotToPosition(m_pivotSubsystem, 3.5).withTimeout(0.25));

    NamedCommands.registerCommand("shootshort", new ParallelCommandGroup(new ConveyorToDuty(m_conveyorSubsystem, 1).withTimeout(0.5), new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(0.5)));
    NamedCommands.registerCommand("shoot", new ParallelCommandGroup(new ConveyorToDuty(m_conveyorSubsystem, 1).withTimeout(1.5), new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(1.5)));

    NamedCommands.registerCommand("intakeshort", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(0.6));
    NamedCommands.registerCommand("intake", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(0.75));
    NamedCommands.registerCommand("intakelong", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(1.5));
    NamedCommands.registerCommand("intakelonglong", new IntakeToDuty(m_intakeSubsystem, 1).withTimeout(1.9));

    NamedCommands.registerCommand("conveyorshort", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.4));
    NamedCommands.registerCommand("conveyor", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.5));
    NamedCommands.registerCommand("conveyorlong", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.6));
    NamedCommands.registerCommand("conveyorlonglong", new ConveyorToDuty(m_conveyorSubsystem, 0.8).withTimeout(0.8));

    NamedCommands.registerCommand("setupnote", new SetupNote(m_conveyorSubsystem, m_intakeSubsystem).withTimeout(0.8));

    NamedCommands.registerCommand("fly", new ShooterToVelocity(m_flywheelSubsystem, Constants.Flywheel.speakerVelo));
    NamedCommands.registerCommand("flytimed", new ShooterToVelocity(m_flywheelSubsystem, Constants.Flywheel.speakerVelo).withTimeout(1));
    NamedCommands.registerCommand("stopfly", new ShooterToDuty(m_flywheelSubsystem, 0).withTimeout(0.05));
    NamedCommands.registerCommand("SetHeading AiNote", new InstantCommand( () -> m_swerveSubsystem.setHeadingState(1)));
    NamedCommands.registerCommand("SetHeading Speaker", new InstantCommand( () -> m_swerveSubsystem.setHeadingState(2)));
    NamedCommands.registerCommand("SetHeading Normal", new InstantCommand( () -> m_swerveSubsystem.setHeadingState(3)));

    
    // NamedCommands.registerCommand("Ai Pickup", new PID_to_game_Piece(m_swerveSubsystem, false, true, false, 2.5));//2.5
    // NamedCommands.registerCommand("Ai Pickup long", new PID_to_game_Piece(m_swerveSubsystem, false, true, false, 3));//2.5
    


    m_swerveSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
        m_swerveSubsystem.applyRequest(() -> drive.withVelocityX(-driva.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driva.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driva.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    configureBindings();
  }

	private boolean getLeftTrigger(XboxController controller) {
		return controller.getLeftTriggerAxis() > Constants.OperatorConstants.triggerDeadzone;
	}

  private boolean getRightTrigger(XboxController controller){
    return controller.getRightTriggerAxis() > Constants.OperatorConstants.triggerDeadzone;
  }
  
  private void configureBindings() {
    //new JoystickButton(driva, 1).onTrue(new ToggleAutoMode(m_pivotSubsystem));
    new JoystickButton(driva, 1).onTrue(new InstantCommand( () -> m_swerveSubsystem.setHeadingState(1)));
    new JoystickButton(driva, 2).onTrue(new InstantCommand( () -> m_swerveSubsystem.setHeadingState(3)));
    new JoystickButton(driva, 3).onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.seedFieldRelative()));
    // new JoystickButton(driva, 4).onTrue(new ToggleAmped(m_swerveSubsystem));
    new JoystickButton(driva, 5).onTrue(new SetupNote(m_conveyorSubsystem, m_intakeSubsystem));
    new JoystickButton(driva, 6).whileTrue(new Shoot(m_conveyorSubsystem, m_intakeSubsystem));
    new Trigger(() -> this.getLeftTrigger(driva)).onTrue(new ShooterToSpeaker(m_flywheelSubsystem));
    new Trigger(() -> this.getRightTrigger(driva)).onTrue(new ShooterToAmp(m_flywheelSubsystem, m_pivotSubsystem));
    // new JoystickButton(driva, 7).onTrue(new TogglePickup(m_swerveSubsystem));
    // new JoystickButton(driva, 8).onTrue(new ToggleAimed(m_swerveSubsystem));
    new POVButton(driva, 90).onTrue(new ToggleFastRotate());

    new JoystickButton(operata, 1).onTrue(new ChangePivotPosition(m_pivotSubsystem, ShooterPositions.SUBWOOFER));
    new JoystickButton(operata, 2).onTrue(new ChangePivotPosition(m_pivotSubsystem, ShooterPositions.PODIUM));
    new JoystickButton(operata, 3).onTrue(new ChangePivotPosition(m_pivotSubsystem, ShooterPositions.UNDER));
    new JoystickButton(operata, 4).onTrue(new ChangePivotPosition(m_pivotSubsystem, ShooterPositions.STAGE));
    m_climberSubsystem.setupController(operata, XboxController.Axis.kLeftY.value);
    
    // new JoystickButton(operata2, 1).onTrue(new SystemCheck(m_ampBarSubsystem, m_climberSubsystem, m_conveyorSubsystem, m_flywheelSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_swerveSubsystem, operata2));


    
    new JoystickButton(shootertesta, 1).whileTrue(new PivotToPosition(m_pivotSubsystem, 1));
    new JoystickButton(shootertesta, 2).onTrue(new ShooterToVelocity(m_flywheelSubsystem, 4000));
    new JoystickButton(shootertesta, 4).whileTrue(new PivotToPosition(m_pivotSubsystem, 5));
    new JoystickButton(shootertesta, 5).onTrue(new ConveyorToPosition(m_conveyorSubsystem, 10));
    new JoystickButton(shootertesta, 6).onTrue(new ConveyorToPosition(m_conveyorSubsystem, 20));
    new JoystickButton(shootertesta, 7).whileTrue(new PivotToDuty(m_pivotSubsystem, -0.08));
    new JoystickButton(shootertesta, 8).whileTrue(new PivotToDuty(m_pivotSubsystem, 0.08));
    new JoystickButton(shootertesta, 9).onTrue(new ResetPivot(m_pivotSubsystem));
    new JoystickButton(shootertesta, 10).whileTrue(new IntakeToDuty(m_intakeSubsystem, 1));

    new JoystickButton(ampBartesta, 1).whileTrue(new AmpBarToDuty(m_ampBarSubsystem, -0.1));
    new JoystickButton(ampBartesta, 4).whileTrue(new AmpBarToDuty(m_ampBarSubsystem, 0.1));
    new JoystickButton(ampBartesta, 5).whileTrue(new AmpBarToDuty(m_ampBarSubsystem, -0.5));
    new JoystickButton(ampBartesta, 6).whileTrue(new AmpBarToDuty(m_ampBarSubsystem, 0.5));
    new JoystickButton(ampBartesta, 7).onTrue(new AmpBarToPosition(m_ampBarSubsystem, Constants.AmpBar.stowPosition));
    new JoystickButton(ampBartesta, 8).onTrue(new AmpBarToPosition(m_ampBarSubsystem, Constants.AmpBar.ampPosition));
  }


  public Command getAutonomousCommand() {
    m_swerveSubsystem.configurePathPlanner();
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new PivotToPosition(m_pivotSubsystem, Constants.Pivot.subwooferPosition).withTimeout(1),
        new ShooterToVelocity(m_flywheelSubsystem, Constants.Flywheel.speakerVelo).withTimeout(1)
      ).withTimeout(1),
      new ConveyorToDuty(m_conveyorSubsystem, 1).withTimeout(0.25),
      new ShooterToDuty(m_flywheelSubsystem, 0).withTimeout(0.05),

      // new ParallelCommandGroup(
        // new ShooterToVelocity(m_flywheelSubsystem, Constants.Flywheel.speakerVelo),
        
        new PathPlannerAuto("New Ai Test")
      // )
    );
  }
}