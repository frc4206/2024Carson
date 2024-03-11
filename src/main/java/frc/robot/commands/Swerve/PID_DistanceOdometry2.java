// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_DistanceOdometry2 extends Command {
	private double rotation;
	private Translation2d translation;
	private boolean fieldRelative;
	private boolean openLoop;
	private boolean isFinished = false;
	
	private SwerveSubsystem s_Swerve;

	private double x_set;
	private double y_set;
	private double yaw_set;

	private double Yaw_Output;

	private double init_time;
	private double current_time;
	private double timeout;
	private boolean stop;

	public PIDController pidx = new PIDController(Constants.Swerve.disOdometryxKP, Constants.Swerve.disOdometryxKI, Constants.Swerve.disOdometryxKD);
	public PIDController pidy = new PIDController(Constants.Swerve.disOdometryYKP, Constants.Swerve.disOdometryYKI, Constants.Swerve.disOdometryYKD);
	public PIDController pidyaw = new PIDController(Constants.Swerve.disOdometryYawKP, Constants.Swerve.disOdometryYawKI, Constants.Swerve.disOdometryYawKD);

	double X_Output;
	double Y_Output;
	double x_error;
	double y_error;
	double yaw_error;

	/** Creates a new PID_DistanceOdometry2. */
	public PID_DistanceOdometry2(SwerveSubsystem s_Swerve, boolean fieldRelative, boolean openLoop, double x_set, double y_set, double yaw_set, double timeout, boolean stop) {
		this.s_Swerve = s_Swerve;
		addRequirements(s_Swerve);
		this.fieldRelative = fieldRelative;
		this.openLoop = openLoop;
		this.x_set = x_set;
		this.y_set = y_set;
		this.yaw_set = yaw_set;
		this.timeout = timeout;
		this.stop = stop;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		init_time = Timer.getFPGATimestamp();
		current_time = 0;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		current_time = Timer.getFPGATimestamp() - init_time;
		double X_Output = 0;
		double Y_Output = 0;
		double x_error = 0;
		double y_error = 0;
		if (DriverStation.getAlliance().get() == Alliance.Blue) {
			X_Output = pidx.calculate(s_Swerve.poseEstimator.getEstimatedPosition().getX(), x_set);
			Y_Output = pidy.calculate(s_Swerve.poseEstimator.getEstimatedPosition().getY(), y_set);
		} else {
			X_Output = pidx.calculate(s_Swerve.poseInvertEstimator.getEstimatedPosition().getX(), x_set);
			Y_Output = pidy.calculate(s_Swerve.poseInvertEstimator.getEstimatedPosition().getY(), y_set);
		}
		

		if (yaw_set == 0) {
			if (s_Swerve.getNominalYaw() > 0 && s_Swerve.getNominalYaw() < 180) {
				Yaw_Output = pidyaw.calculate(s_Swerve.getNominalYaw(), 0);
			} else {
				Yaw_Output = pidyaw.calculate(s_Swerve.getNominalYaw(), yaw_set);
			}
			x_error = Math.abs(s_Swerve.poseEstimator.getEstimatedPosition().getX() - Math.abs(x_set));
			y_error = Math.abs(s_Swerve.poseEstimator.getEstimatedPosition().getY() - Math.abs(y_set));
			yaw_error = Math.abs(s_Swerve.getNominalYaw()) - (yaw_set);
		} else {
			X_Output = pidx.calculate(s_Swerve.swerveInvertOdometry.getPoseMeters().getX(), x_set);
			Y_Output = pidy.calculate(s_Swerve.swerveInvertOdometry.getPoseMeters().getY(), y_set);
			if (yaw_set == 0) {
				if (s_Swerve.getNominalYawInverted() > 0 && s_Swerve.getNominalYawInverted() < 180) {
					Yaw_Output = pidyaw.calculate(s_Swerve.getNominalYawInverted(), 0);
				} else {
					Yaw_Output = pidyaw.calculate(s_Swerve.getNominalYawInverted(), 360);
				}
			} else if(Math.abs(s_Swerve.getNominalYawInverted() - yaw_set) > 180) {
				if (yaw_set < 90) {
					Yaw_Output = pidyaw.calculate(s_Swerve.getNominalYawInverted(), 360+yaw_set);
				} else if (yaw_set > 270) {
					Yaw_Output = pidyaw.calculate(s_Swerve.getNominalYawInverted(), 0-yaw_set);
				}
			} else {
				Yaw_Output = pidyaw.calculate(s_Swerve.getNominalYawInverted(), yaw_set);
			}
			x_error = Math.abs(s_Swerve.swerveInvertOdometry.getPoseMeters().getX() - Math.abs(x_set));
			y_error = Math.abs(s_Swerve.swerveInvertOdometry.getPoseMeters().getY() - Math.abs(y_set));


			yaw_error = Math.abs(s_Swerve.getNominalYawInverted()) - (yaw_set);
		}

		
		if (DriverStation.getAlliance().get() == Alliance.Blue) {
			x_error = Math.abs(s_Swerve.poseEstimator.getEstimatedPosition().getX() - Math.abs(x_set));
			y_error = Math.abs(s_Swerve.poseEstimator.getEstimatedPosition().getY() - Math.abs(y_set));
		} else {
			x_error = Math.abs(s_Swerve.poseInvertEstimator.getEstimatedPosition().getX() - Math.abs(x_set));
			y_error = Math.abs(s_Swerve.poseInvertEstimator.getEstimatedPosition().getY() - Math.abs(y_set));
		}


		
		if ((x_error < Constants.Swerve.disOdometryMaxPosError && y_error < Constants.Swerve.disOdometryMaxPosError && yaw_error < Constants.Swerve.disOdometryMaxRotationError) || current_time > timeout){
			isFinished();
		}

		 if (DriverStation.getAlliance().get() == Alliance.Red){
		 	X_Output = -X_Output;

		 	Y_Output = -Y_Output;
			Yaw_Output = -Yaw_Output;

		 	Y_Output =  0;
			Yaw_Output = 0;

		 }

    translation = new Translation2d(X_Output, Y_Output).times(Constants.Swerve.maxSpeed);
		rotation = Yaw_Output;

		s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (this.stop) {
			s_Swerve.drive(new Translation2d(), 0, true, true);
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return isFinished;
	}
}