// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {

  public CANSparkFlex pivotMotor = new CANSparkFlex(Constants.Shooter.ShooterPivotID, MotorType.kBrushless);
  public SparkPIDController pivotController;

  public RelativeEncoder relPivotEnc;

  double [][] AngleData = { {0,0}, {1,1} };

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
    pivotController = pivotMotor.getPIDController();


    relPivotEnc = pivotMotor.getEncoder();


    pivotController.setFeedbackDevice(relPivotEnc);
    pivotController.setP(Constants.Shooter.pivotKP);
    pivotController.setI(Constants.Shooter.pivotKI);
    pivotController.setD(Constants.Shooter.pivotKD);
    pivotController.setFF(Constants.Shooter.pivotFF);
    pivotController.setSmartMotionMaxVelocity(Constants.Shooter.pivotMaxVel, Constants.Shooter.pivotMaxVelID);
    pivotController.setSmartMotionMinOutputVelocity(Constants.Shooter.pivotMinVel, Constants.Shooter.pivotMinVelID);
    pivotController.setSmartMotionMaxAccel(Constants.Shooter.pivotMaxAccel, Constants.Shooter.pivotMaxAccelID);
    pivotController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.pivotAllowedError, Constants.Shooter.pivotAllowedErrorID);
 
    pivotMotor.setSmartCurrentLimit(40);
  }

  public void motorPivot(double pivotSpeed) {
    // pivotMotor.set(pivotSpeed);
  }

  public void setPos(double xCoordinate) {
    pivotController.setReference(xCoordinate, CANSparkFlex.ControlType.kPosition);
  }

  public void runMotor (double speed) {
    pivotMotor.set(speed);
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
