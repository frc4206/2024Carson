// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class PivotSubsystem extends SubsystemBase {

  public CANSparkFlex pivotMotor = new CANSparkFlex(Constants.Shooter.shooterPivotID, MotorType.kBrushless);
  public RelativeEncoder pivotEncoder;
  public SparkPIDController pivotPIDController;

  double[][] angleData = {{4.65, 1.03}, {4.04, 1.86}, {3.89, 1.69}, {3.12, 2.77}, {2.90, 3.21}, {2.49, 4.07}, {2.18, 5.00}};
  InterpolatingTreeTableSubsystem angleTree;

  public enum ShooterPositions {
    SUBWOOFER,
    PODIUM,
    AMPLIFIER,
    WING;
  }

  public ShooterPositions position;

  public PivotSubsystem() {
    angleTree = new InterpolatingTreeTableSubsystem(angleData);

    pivotMotor.restoreFactoryDefaults();
    pivotEncoder = pivotMotor.getEncoder();
    pivotPIDController = pivotMotor.getPIDController();
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setClosedLoopRampRate(0.25);
    pivotMotor.setSmartCurrentLimit(40);
    pivotPIDController.setFeedbackDevice(pivotEncoder);
    pivotPIDController.setP(Constants.Shooter.pivotKP);
    pivotPIDController.setI(Constants.Shooter.pivotKI);
    pivotPIDController.setIZone(Constants.Shooter.pivotKIZone);
    pivotPIDController.setD(Constants.Shooter.pivotKD);
    pivotPIDController.setSmartMotionMaxVelocity(Constants.Shooter.pivotMaxVel, 0);
    pivotPIDController.setSmartMotionMinOutputVelocity(Constants.Shooter.pivotMinVel, 0);
    pivotPIDController.setSmartMotionMaxAccel(Constants.Shooter.pivotMaxAccel, 0);
    pivotPIDController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.pivotAllowedError, 0);
  }

  public void resetPivot(){
    pivotEncoder.setPosition(0);
  }

  public void runPivot(double speed) {
    pivotMotor.set(speed);
  }
  
  public void setPosition(double angle) {
    pivotPIDController.setReference(angle, CANSparkFlex.ControlType.kPosition);
  }

  public void autoAdjust(double distFromSpeaker){
    double newAngle = angleTree.getInterpolatedValue(distFromSpeaker);
    setPosition(newAngle);
  }

  // Set shooter position relative to field (where am I shooting?)
  public void setFieldRelativePosition() {
    switch(position) {
      case SUBWOOFER:
        pivotPIDController.setReference(Constants.Shooter.AngleSUBWOOFERPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case PODIUM:
        pivotPIDController.setReference(Constants.Shooter.AnglePODIUMPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case AMPLIFIER:
        pivotPIDController.setReference(Constants.Shooter.AngleAMPLIFIERPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case WING:
        pivotPIDController.setReference(Constants.Shooter.AngleWINGPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot position", pivotEncoder.getPosition());
    if (!GlobalVariables.shooterAutomatic){
      // setFieldRelativePosition();
    }
  }
}
