// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveOLD;

public class PID_Distance_Object_Detection extends Command {
    /** Creates a new PID_DistanceOdometry2. */
    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    private boolean isFinished;
    
    private SwerveOLD s_Swerve;

    private double x_set;
    private double y_set;
    private double yaw_set;

    private double init_time;
    private double current_time;
    private double timeout;
    
    public PIDController pidx = new PIDController(Constants.Swerve.objDetectxKP, Constants.Swerve.objDetectxKI, Constants.Swerve.objDetectxKD);
    public PIDController pidy = new PIDController(Constants.Swerve.objDetectYKP, Constants.Swerve.objDetectYKI, Constants.Swerve.objDetectYKD);
    public PIDController pidyaw = new PIDController(Constants.Swerve.objDetectYawKP, Constants.Swerve.objDetectYawKI, Constants.Swerve.objDetectYawKD);

    double Go_to_Target_Distance;

    public PID_Distance_Object_Detection(SwerveOLD s_Swerve, boolean fieldRelative, boolean openLoop, double x_set, double y_set, double yaw_set, double timeout, double Go_to_Target_Distance) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.x_set = x_set;
        this.y_set = y_set;
        this.yaw_set = yaw_set;
        this.timeout = timeout;
        this.Go_to_Target_Distance = Go_to_Target_Distance;
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
        fieldRelative = true;
        
        double X_Output = 0;
        double Y_Output = 0;
        double Yaw_Output = 0;

        double x_error = Math.abs(s_Swerve.poseEstimator.getEstimatedPosition().getX()) - Math.abs(x_set);
        double y_error = Math.abs(s_Swerve.poseEstimator.getEstimatedPosition().getX()) - Math.abs(y_set);
        double yaw_error = Math.abs(s_Swerve.getYaw().getDegrees()) - (yaw_set);
        double distanceToGamePiece = (Limelight.limelightshooter.HasTarget() == 1) ? Limelight.limelightManger.cameraList[1].GetDistanceToGamePiece() : 0;

        if(distanceToGamePiece < Go_to_Target_Distance && distanceToGamePiece != 0) {
            rotation = pidyaw.calculate(Limelight.limelightshooter.limelightTable.getEntry("tx").getDouble(0), 0);
        } else {
            X_Output = pidx.calculate(s_Swerve.poseEstimator.getEstimatedPosition().getX(), x_set);
            Y_Output = pidy.calculate(s_Swerve.poseEstimator.getEstimatedPosition().getY(), y_set);
            Yaw_Output = pidyaw.calculate(s_Swerve.getYaw().getDegrees(), yaw_set);
        }

        if (x_error < Constants.Swerve.objDetectMaxPosError && y_error < Constants.Swerve.objDetectMaxPosError && yaw_error < Constants.Swerve.objDetectMaxRotationError) {
            System.out.println("terminated on error");
            isFinished = true;
            isFinished();
        }
        if (current_time > timeout) {
            System.out.println("terminated on timeout");
            isFinished = true;
            isFinished();
        }

        translation = new Translation2d(X_Output, Y_Output).times(Constants.Swerve.maxSpeed).times(s_Swerve.currPercent);
        rotation = Yaw_Output;

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
