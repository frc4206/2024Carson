// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
//e
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.ShooterPositions;

public class PivotCommand extends Command {
	public PivotSubsystem m_pivotSubsystem;
	public double pivotSpeed;
	public ShooterPositions positon;

	/** Creates a new PivotCommand. */
	public PivotCommand(PivotSubsystem pivotSubsystem, double pivSpeed) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_pivotSubsystem = pivotSubsystem;
		pivotSpeed = pivSpeed;
		addRequirements(m_pivotSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Which one do we use and when?
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_pivotSubsystem.setPos(pivotSpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
