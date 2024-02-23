// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_DistanceOdometry2 extends Command {
  /** Creates a new PID_DistanceOdometry2. */
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  private boolean isFinished;
  
  private SwerveSubsystem s_Swerve;

  private double x_set;
  private double y_set;
  private double yaw_set;

  private double init_time;
  private double current_time;
  private double timeout;

  public PIDController pidx = new PIDController(Constants.Swerve.disOdometryxKP, Constants.Swerve.disOdometryxKI, Constants.Swerve.disOdometryxKD);
  public PIDController pidy = new PIDController(Constants.Swerve.disOdometryYKP, Constants.Swerve.disOdometryYKI, Constants.Swerve.disOdometryYKD);
  //public PIDController pidyaw = new PIDController(Constants.Swerve.disOdometryYawKP, Constants.Swerve.disOdometryYawKI, Constants.Swerve.disOdometryYawKD);

  public PID_DistanceOdometry2(SwerveSubsystem s_Swerve, boolean fieldRelative, boolean openLoop, double x_set, double y_set, double yaw_set, double timeout) {
      this.s_Swerve = s_Swerve;
      addRequirements(s_Swerve);
      this.fieldRelative = fieldRelative;
      this.openLoop = openLoop;
      this.x_set = x_set;
      this.y_set = y_set;
      this.yaw_set = yaw_set;
      this.timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    init_time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_time = Timer.getFPGATimestamp() - init_time;
    double X_Output = pidx.calculate(s_Swerve.swerveOdometry.getPoseMeters().getX(), x_set);
    double Y_Output = pidy.calculate(s_Swerve.swerveOdometry.getPoseMeters().getY(), y_set);
    //double Yaw_Output = pidyaw.calculate(s_Swerve.getYaw().getDegrees(), yaw_set);

    double x_error = Math.abs(s_Swerve.swerveOdometry.getPoseMeters().getX()) - Math.abs(x_set);
    double y_error = Math.abs(s_Swerve.swerveOdometry.getPoseMeters().getY()) - Math.abs(y_set);
    double yaw_error = Math.abs(s_Swerve.getYaw().getDegrees()) - (yaw_set);
    SmartDashboard.putNumber("Current time", current_time);

    if ((x_error < Constants.Swerve.disOdometryMaxPosError && y_error < Constants.Swerve.disOdometryMaxPosError && yaw_error < Constants.Swerve.disOdometryMaxRotationError)|| current_time > timeout) {
      isFinished = true;
      isFinished();
    }


    SmartDashboard.putNumber("pid output", X_Output);

    translation = new Translation2d(X_Output, Y_Output).times(Constants.Swerve.maxSpeed).times(s_Swerve.currPercent);
    //rotation = Yaw_Output;

    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}