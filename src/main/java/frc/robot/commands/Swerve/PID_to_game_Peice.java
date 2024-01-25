// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_to_game_Peice extends Command {
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
  private double current_time;
  private double timeout;

  public PIDController pidx = new PIDController(0.0002, 0, 0);
  public PIDController pidy = new PIDController(0.000, 0, 0);
  public PIDController pidyaw = new PIDController(0.0002, 0, 0);

  public PID_to_game_Peice(SwerveSubsystem s_Swerve, boolean fieldRelative, boolean openLoop, boolean advanced_setpoints, double timeout) {
      this.s_Swerve = s_Swerve;
      addRequirements(s_Swerve);
      this.fieldRelative = fieldRelative;
      this.openLoop = openLoop;
      this.advanced_setpoints = advanced_setpoints;
      this.timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_time = Timer.getFPGATimestamp() - init_time;
    double[] OdometryArray = {s_Swerve.getPose().getX(), s_Swerve.getPose().getY()};

    double[] gamePiecePos = s_Swerve.limelight.limelightfront.GetGamePiecePosition(OdometryArray, s_Swerve.getYaw().getDegrees());
    x_set = gamePiecePos[0];
    y_set = gamePiecePos[1];
    rotation = 0;

    double X_Output = 0;
    double Y_Output = 0;

    SmartDashboard.putNumber("x game peice pos", x_set);
    SmartDashboard.putNumber("y game peice pos", y_set);

    if (advanced_setpoints) {
      fieldRelative = true;
      X_Output = pidx.calculate(s_Swerve.swerveOdometry.getPoseMeters().getX(), x_set);
      Y_Output = pidy.calculate(s_Swerve.swerveOdometry.getPoseMeters().getY(), y_set);
    } else {
      //fieldRelative = false;
      X_Output = (s_Swerve.limelight.limelightManger.cameraList[1].GetDistanceToGamePiece() == 0) ? X_Output : pidx.calculate(s_Swerve.limelight.limelightManger.cameraList[1].GetDistanceToGamePiece());
      rotation = pidyaw.calculate(s_Swerve.limelight.limelightfront.limelightTable.getEntry("tx").getDouble(0), 0);
      SmartDashboard.putNumber("angle", s_Swerve.limelight.limelightfront.limelightTable.getEntry("tx").getDouble(0));
      SmartDashboard.putNumber("distance to game peice in command", s_Swerve.limelight.limelightManger.cameraList[1].GetDistanceToGamePiece());
    }
    
    SmartDashboard.putNumber("pid output piece ", X_Output);

    translation = new Translation2d(X_Output, Y_Output).times(Constants.Swerve.maxSpeed).times(s_Swerve.currPercent);

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
