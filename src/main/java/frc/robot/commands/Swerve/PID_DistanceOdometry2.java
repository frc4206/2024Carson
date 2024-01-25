// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
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
  
  private SwerveSubsystem s_Swerve;
  private Joystick controller;
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;

  private double x_set;
  private double y_set;

  public PIDController pidx = new PIDController(0.0002, 0, 0);
   public PIDController pidy = new PIDController(0.0002, 0, 0);

  public PID_DistanceOdometry2(SwerveSubsystem s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop, double x_set, double y_set) {
      this.s_Swerve = s_Swerve;
      addRequirements(s_Swerve);

      this.controller = controller;
      this.translationAxis = translationAxis;
      this.strafeAxis = strafeAxis;
      this.rotationAxis = rotationAxis;
      this.fieldRelative = fieldRelative;
      this.openLoop = openLoop;
      this.x_set = x_set;
      this.y_set = y_set;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double X_Output = pidx.calculate(s_Swerve.swerveOdometry.getPoseMeters().getX(), x_set);
    double Y_Output = pidy.calculate(s_Swerve.swerveOdometry.getPoseMeters().getY(), y_set);
    SmartDashboard.putNumber("pid output", X_Output);

    translation = new Translation2d(X_Output, Y_Output).times(Constants.Swerve.maxSpeed).times(s_Swerve.currPercent);
    rotation = 0;

    s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
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
