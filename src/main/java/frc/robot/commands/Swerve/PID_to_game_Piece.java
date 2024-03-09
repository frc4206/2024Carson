// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_to_game_Piece extends Command {
	/** Creates a new PID_DistanceOdometry2. */
	private double rotation;
	private Translation2d translation;
	private boolean fieldRelative;
	private boolean openLoop;
	
	private SwerveSubsystem s_Swerve;

	private double x_set;
	private double y_set;

	private boolean advanced_setpoints;

	private double init_time;
	double current_time;
	double timeout;
	boolean fin = false;

	public PIDController pidx = new PIDController(Constants.Swerve.toGamePiecexKP, Constants.Swerve.toGamePiecexKI, Constants.Swerve.toGamePiecexKD);
	public PIDController pidy = new PIDController(Constants.Swerve.toGamePieceYKP, Constants.Swerve.toGamePieceYKI, Constants.Swerve.toGamePieceYKD);
	public PIDController pidyaw = new PIDController(Constants.Swerve.toGamePieceYawKP, Constants.Swerve.toGamePieceYawKI, Constants.Swerve.toGamePieceYawKD);

	public PID_to_game_Piece(SwerveSubsystem s_Swerve, boolean fieldRelative, boolean openLoop, boolean advanced_setpoints, double timeout) {
		this.s_Swerve = s_Swerve;
		addRequirements(s_Swerve);
		this.fieldRelative = fieldRelative;
		this.openLoop = openLoop;
		this.advanced_setpoints = advanced_setpoints;
		this.timeout = timeout;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		init_time = Timer.getFPGATimestamp();
		fin = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		current_time = Timer.getFPGATimestamp() - init_time;
		double[] OdometryArray = {s_Swerve.poseEstimator.getEstimatedPosition().getX(), s_Swerve.poseEstimator.getEstimatedPosition().getY()};

		double[] gamePiecePos = Limelight.limelightintake.GetGamePiecePosition(OdometryArray, s_Swerve.getYaw().getDegrees());
		x_set = gamePiecePos[0];
		y_set = gamePiecePos[1];
		rotation = 0;

		double X_Output = 0.2;
		double Y_Output = 0;

		SmartDashboard.putNumber("x game piece pos", x_set);
		SmartDashboard.putNumber("y game piece pos", y_set);

		if(advanced_setpoints) {
			fieldRelative = true;
			X_Output = pidx.calculate(s_Swerve.swerveOdometry.getPoseMeters().getX(), x_set);
			Y_Output = pidy.calculate(s_Swerve.swerveOdometry.getPoseMeters().getY(), y_set);
		} else {
			//fieldRelative = false;
			//X_Output = (Limelight.limelightintake.HasTarget() == 0) ? X_Output : pidx.calculate(Limelight.limelightManger.cameraList[1].GetDistanceToGamePiece());
			SmartDashboard.putNumber("distance to game piece in command", Limelight.limelightManger.cameraList[1].GetDistanceToGamePiece());
		}
		//Limelight.limelightintake.limelightTable.getEntry("tx").getDouble(0)
		SmartDashboard.putNumber("angle", SmartDashboard.getNumber("tx", 10));
		rotation = pidyaw.calculate(Limelight.limelightintake.limelightTable.getEntry("tx").getDouble(0), 0);
		SmartDashboard.putNumber("pid output piece ", X_Output);

		translation = new Translation2d(X_Output, 0).times(Constants.Swerve.maxSpeed).times(s_Swerve.currPercent);

		s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
		if (current_time > timeout) {
			fin = true;
			isFinished();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return fin;
	}
}
// Polo