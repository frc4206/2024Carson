// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class FlywheelSubsystem extends SubsystemBase {
  private CANSparkFlex upperFlyMotor = new CANSparkFlex(Constants.Shooter.shooterLeadMotorID, MotorType.kBrushless); 
  private CANSparkFlex lowerFlyMotor = new CANSparkFlex(Constants.Shooter.shooterFollowerID, MotorType.kBrushless);
  private RelativeEncoder upperFlyEncoder = upperFlyMotor.getEncoder();
  private RelativeEncoder lowerFlyEncoder = lowerFlyMotor.getEncoder();
  private SparkPIDController upperFlyPIDController = upperFlyMotor.getPIDController();
  private SparkPIDController lowerFlyPIDController = lowerFlyMotor.getPIDController();

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem() {
    // upperFlyMotor.restoreFactoryDefaults();
    lowerFlyMotor.restoreFactoryDefaults();
    
    upperFlyMotor.setInverted(true);
    lowerFlyMotor.setInverted(true);
    upperFlyMotor.setIdleMode(IdleMode.kCoast);
    lowerFlyMotor.setIdleMode(IdleMode.kCoast);

    upperFlyPIDController.setFeedbackDevice(upperFlyEncoder);
    upperFlyPIDController.setP(Constants.Shooter.topFlyWheelKP);
    upperFlyPIDController.setI(Constants.Shooter.topFlyWheelKI);
    upperFlyPIDController.setIZone(0);
    upperFlyPIDController.setD(Constants.Shooter.topFlyWheelKD);
    upperFlyPIDController.setOutputRange(-1, 1, 0);
    upperFlyPIDController.setSmartMotionMaxVelocity(Constants.Shooter.topFlyWheelMaxVel, 0);
    upperFlyPIDController.setSmartMotionMaxAccel(Constants.Shooter.topFlyWheelMaxAccel, 0);
    upperFlyPIDController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.topFlyWheelAllowedError, 0);

    lowerFlyPIDController.setFeedbackDevice(lowerFlyEncoder);
    lowerFlyPIDController.setP(Constants.Shooter.bottomFlyWheelKP);
    lowerFlyPIDController.setI(Constants.Shooter.bottomFlyWheelKI);
    lowerFlyPIDController.setIZone(0);
    lowerFlyPIDController.setD(Constants.Shooter.bottomFlyWheelKD);
    lowerFlyPIDController.setOutputRange(-1, 1, 0);
    lowerFlyPIDController.setSmartMotionMaxVelocity(Constants.Shooter.bottomFlyWheelMaxVel, 0);
    lowerFlyPIDController.setSmartMotionMaxAccel(Constants.Shooter.bottomFlyWheelMaxAccel, 0);
    lowerFlyPIDController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.bottomFlyWheelAllowedError, 0);

    upperFlyMotor.burnFlash();
    lowerFlyMotor.burnFlash();
  }

  public void shootAmp(){
    upperFlyMotor.set(1);
    lowerFlyMotor.set(-1);
  }

  public boolean shooterAtVelocity(double setVelocity){
    return (
      (Math.abs(upperFlyEncoder.getVelocity() - setVelocity) < 100) &&
      (Math.abs(lowerFlyEncoder.getVelocity() - setVelocity) < 50)
    );
  }

  public void setVelocity(double setVelocity) {
    upperFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
    lowerFlyPIDController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
  }

  public void percentShooter(double percent) {
    upperFlyMotor.set(percent);
    lowerFlyMotor.set(percent);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("topVelo", upperFlyEncoder.getVelocity());
    SmartDashboard.putNumber("bottomVelo", lowerFlyEncoder.getVelocity());
  }
}