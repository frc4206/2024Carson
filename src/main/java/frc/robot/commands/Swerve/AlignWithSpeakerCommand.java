// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveOLD;

public class AlignWithSpeakerCommand extends Command {
	/** Creates a new PID_DistanceOdometry2. */
	private double rotation;
	private Translation2d translation;
	
	private SwerveOLD s_Swerve;

	public PIDController pidyaw = new PIDController(0.01, Constants.Swerve.toGamePieceYawKI, Constants.Swerve.toGamePieceYawKD);

	public AlignWithSpeakerCommand(SwerveOLD s_Swerve) {
		this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        for(SwerveModule mod : s_Swerve.mSwerveMods) {
            mod.mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
        }

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		rotation = pidyaw.calculate(Limelight.limelightshooter.limelightTable.getEntry("tx").getDouble(0), 0);

		translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed);

		s_Swerve.drive(translation, rotation, true, true);

		if (Math.abs(Limelight.limelightshooter.limelightTable.getEntry("tx").getDouble(0)) < 3) {
			end(true);
		}

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
// Polo