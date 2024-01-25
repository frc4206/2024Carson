// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  public CANSparkFlex pivotMotor = new CANSparkFlex(13, MotorType.kBrushless);
  public SparkPIDController pivotController;

  public RelativeEncoder relPivotEnc;

  // To finish on Monday
  public enum ShooterPositions {
    SUBWOOFER,
    PODIUM,
    AMPLIFIER,
    WING;
  }

  private ShooterPositions position;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(ShooterPositions position) {
    pivotMotor.restoreFactoryDefaults();

    pivotController.setFeedbackDevice(relPivotEnc);
    pivotController.setP(0.02);
    pivotController.setI(9e-8);
    pivotController.setD(0.0);
    pivotController.setFF(0.0);
    pivotController.setSmartMotionMaxVelocity(1, 0);
    pivotController.setSmartMotionMinOutputVelocity(-1, 0);
    pivotController.setSmartMotionMaxAccel(100, 0);
    pivotController.setSmartMotionAllowedClosedLoopError(5, 0);

    pivotController = pivotMotor.getPIDController();

    relPivotEnc = pivotMotor.getEncoder();
  }

  public void motorPivot(double pivotSpeed) {
    // pivotMotor.set(pivotSpeed);
  }

  public void setPos(double xCoordinate, double yCoordinate) {
    //pivotController.setReference(xCoordinate, yCoordinate, CANSparkFlex.ControlType.kPosition);
  }

  // Set shooter position relative to field (where am I shooting?)
  public void setFieldRelativePosition(int degrees, ShooterPositions position) {
    switch(position) {
      case SUBWOOFER:
        pivotController.setReference(0 /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case PODIUM:
        pivotController.setReference(0 /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case AMPLIFIER:
        pivotController.setReference(0 /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
      case WING:
        pivotController.setReference(0 /*PLACEHOLDER!*/, CANSparkFlex.ControlType.kPosition);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}