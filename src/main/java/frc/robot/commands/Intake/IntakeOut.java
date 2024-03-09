// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRunBothDownCommand.java
package frc.robot.commands.Climber;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberRunBothDownCommand extends Command {
	/** Creates a new ClimberRunBothMotorsDownCommand. */
	ClimberSubsystem m_climberSubsystem; 

	public ClimberRunBothDownCommand(ClimberSubsystem climber) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_climberSubsystem = climber; 
		addRequirements(m_climberSubsystem);
========
package frc.robot.commands.Intake;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOut extends Command {
	private IntakeSubsystem m_intakeMotor;

	/** Creates a new IntakeOut. */
	public IntakeOut(IntakeSubsystem intakeMotor) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_intakeMotor = intakeMotor; 
		addRequirements(m_intakeMotor);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Intake/IntakeOut.java
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/Climber/ClimberRunBothDownCommand.java
		m_climberSubsystem.climbDOWN(); 
========
		m_intakeMotor.intakeGo(-0.5);
>>>>>>>> b7bd597c3eca3ecef4cb29ac3f28cb0a3b757362:src/main/java/frc/robot/commands/Intake/IntakeOut.java
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
