// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithSpeakerCommand extends Command {
	/** Creates a new PID_DistanceOdometry2. */
	private double rotation;
	private Translation2d translation;
	private boolean fieldRelative;
	private boolean openLoop;
	
	private SwerveSubsystem s_Swerve;

	private XboxController controller;
    private int translationAxis;
    private int strafeAxis;


	public PIDController pidyaw = new PIDController(Constants.Swerve.toGamePieceYawKP, Constants.Swerve.toGamePieceYawKI, Constants.Swerve.toGamePieceYawKD);

	public AlignWithSpeakerCommand(SwerveSubsystem s_Swerve, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
		this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

		this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
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
		double yAxisDeadzoned;
        double xAxisDeadzoned;
        double yAxis = (-controller.getRawAxis(translationAxis)*Constants.Swerve.translationMultiplier);
        double xAxis = (-controller.getRawAxis(strafeAxis)*Constants.Swerve.translationMultiplier);

        yAxisDeadzoned = (Math.abs(yAxis) < Constants.OperatorConstants.stickDeadband) ? 0 : s_Swerve.map(Math.abs(yAxis), Constants.OperatorConstants.stickDeadband, 1.0, 0.0, 1.0);
        yAxisDeadzoned = yAxis >= 0.0 ? yAxisDeadzoned : -yAxisDeadzoned;
        yAxisDeadzoned = yAxisDeadzoned * yAxisDeadzoned; //(Math.cos(Math.PI*(yAxisDeadzoned + 1.0d)/2.0d)) + 0.5d;
        yAxisDeadzoned = yAxis >= 0.0 ? yAxisDeadzoned : -yAxisDeadzoned; 

        xAxisDeadzoned = (Math.abs(xAxis) < Constants.OperatorConstants.stickDeadband) ? 0 : s_Swerve.map(Math.abs(xAxis), Constants.OperatorConstants.stickDeadband, 1.0, 0.0, 1.0);
        xAxisDeadzoned = xAxis >= 0.0 ? xAxisDeadzoned : -xAxisDeadzoned;
        xAxisDeadzoned = xAxisDeadzoned * xAxisDeadzoned; //(Math.cos(Math.PI*(xAxisDeadzoned + 1.0d)/2.0d)) + 0.5d;
        xAxisDeadzoned = xAxis >= 0.0 ? xAxisDeadzoned : -xAxisDeadzoned;

		SmartDashboard.putNumber("distance to game piece in command", Limelight.limelightManger.cameraList[1].GetDistanceToGamePiece());
		rotation = pidyaw.calculate(Limelight.limelightshooter.limelightTable.getEntry("tx").getDouble(0), 0);

		translation = new Translation2d(yAxisDeadzoned, xAxisDeadzoned).times(Constants.Swerve.maxSpeed);

		s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

		SmartDashboard.putNumber("Distance to Speaker", Math.sqrt((Limelight.limelightshooter.aprilTagResult[0] * Limelight.limelightshooter.aprilTagResult[0]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1])));

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