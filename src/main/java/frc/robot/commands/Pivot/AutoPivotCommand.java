// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivotCommand extends Command {
	public PivotSubsystem m_pivotSubsystem;
	public double m_pivotPosition;

	/** Creates a new PivotCommand. */
	public AutoPivotCommand(PivotSubsystem pivotSubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_pivotSubsystem = pivotSubsystem;
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
		m_pivotSubsystem.setPosition(SmartDashboard.getNumber("desired angle", 8));
		if (m_pivotSubsystem.pivotEncoder.getPosition() < 0.1) {
		end(true);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("ended");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
