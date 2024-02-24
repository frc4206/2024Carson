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

  public CANSparkFlex pivotMotor = new CANSparkFlex(Constants.Pivot.pivotMotorID, MotorType.kBrushless);
  public RelativeEncoder relPivotEnc;
  public SparkPIDController pivotController;

  double[][] angleData = {{4.65, 1.03}, {4.04, 1.86}, {3.89, 1.69}, {3.12, 2.77}, {2.90, 3.21}, {2.49, 4.07}, {2.18, 5.00}};
  InterpolatingTreeTableSubsystem angleTree;

  // To finish on Monday
  public enum ShooterPositions {
    SUBWOOFER,
    PODIUM,
    AMPLIFIER,
    WING;
  }

  public ShooterPositions position;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    angleTree = new InterpolatingTreeTableSubsystem(angleData);

    pivotMotor.restoreFactoryDefaults();
    relPivotEnc = pivotMotor.getEncoder();
    pivotController = pivotMotor.getPIDController();
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setClosedLoopRampRate(0.5);
    pivotMotor.setSmartCurrentLimit((int) Constants.Pivot.pivotCurrLimit);
    pivotController.setFeedbackDevice(relPivotEnc);
    pivotController.setP(Constants.Pivot.pivotKP);
    pivotController.setI(Constants.Pivot.pivotKI);
    pivotController.setD(Constants.Pivot.pivotKD);
    pivotController.setFF(Constants.Pivot.pivotFF);
    pivotController.setSmartMotionMaxVelocity(Constants.Pivot.pivotMaxVel, Constants.Pivot.pivotMaxVelID);
    pivotController.setSmartMotionMinOutputVelocity(Constants.Pivot.pivotMinVel, Constants.Pivot.pivotMinVelID);
    pivotController.setSmartMotionMaxAccel(Constants.Pivot.pivotMaxAccel, Constants.Pivot.pivotMaxAccelID);
    pivotController.setSmartMotionAllowedClosedLoopError(Constants.Pivot.pivotAllowedError, Constants.Pivot.pivotAllowedErrorID);
  }

  public void runMotor(double speed) {
    pivotMotor.set(speed);
  }
  
  public void setPos(double xCoordinate) {
    pivotController.setReference(xCoordinate, CANSparkFlex.ControlType.kPosition);
  }

  public void autoAdjust(double distFromSpeaker){
    double newAngle = angleTree.getInterpolatedValue(distFromSpeaker);
    setPos(newAngle);
  }

  // Set shooter position relative to field (where am I shooting?)
  public void setFieldRelativePosition() {
    switch(position) {
      case SUBWOOFER:
        pivotController.setReference(Constants.Pivot.AngleSUBWOOFERPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case PODIUM:
        pivotController.setReference(Constants.Pivot.AnglePODIUMPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case AMPLIFIER:
        pivotController.setReference(Constants.Pivot.AngleAMPLIFIERPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case WING:
        pivotController.setReference(Constants.Pivot.AngleWINGPosition /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot position", relPivotEnc.getPosition());
    if (!GlobalVariables.shooterAutomatic){
      // setFieldRelativePosition();
    }
  }
}
