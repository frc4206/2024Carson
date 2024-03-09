// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/RunServoLeftCommand.java
package frc.robot.commands.Climber.ClimberLeft;

========
package frc.robot.commands.Intake;
import frc.robot.subsystems.IntakeSubsystem;
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Intake/IntakeIn.java
import edu.wpi.first.wpilibj2.command.Command;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/RunServoLeftCommand.java
public class RunServoLeftCommand extends Command {
	public ClimberSubsystem m_climberSubsystem;
	public double servoPosition;
	
	/** Creates a new RunServoCommand. */
	public RunServoLeftCommand(ClimberSubsystem climber_L, double pos) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_climberSubsystem = climber_L;
		servoPosition = pos;
		addRequirements(m_climberSubsystem);
========
public class IntakeIn extends Command {
	private IntakeSubsystem m_intakeMotor;

	/** Creates a new IntakeGo. */
	public IntakeIn(IntakeSubsystem intakeMotor) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_intakeMotor = intakeMotor; 
		addRequirements(m_intakeMotor);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Intake/IntakeIn.java
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberLeft/RunServoLeftCommand.java
		m_climberSubsystem.setPositionLeft(servoPosition);
========
		m_intakeMotor.intakeGo(0.5);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Intake/IntakeIn.java
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_intakeMotor.intakeGo(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
