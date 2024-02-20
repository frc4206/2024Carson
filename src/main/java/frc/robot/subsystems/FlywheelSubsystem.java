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
  private SparkPIDController upperFlyController = upperFlyMotor.getPIDController();
  private SparkPIDController lowerFlyController = lowerFlyMotor.getPIDController();

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem() {
    upperFlyMotor.restoreFactoryDefaults();
    lowerFlyMotor.restoreFactoryDefaults();
    
    upperFlyMotor.setInverted(true);
    lowerFlyMotor.setInverted(true);
    upperFlyMotor.setIdleMode(IdleMode.kCoast);
    lowerFlyMotor.setIdleMode(IdleMode.kCoast);

    upperFlyController.setFeedbackDevice(upperFlyEncoder);
    upperFlyController.setP(Constants.Shooter.flyWheelKP);
    upperFlyController.setI(Constants.Shooter.flyWheelKI);
    upperFlyController.setD(Constants.Shooter.flyWheelKD);
    upperFlyController.setIZone(Constants.Shooter.flyWheelIZone); 
    upperFlyController.setFF(Constants.Shooter.flyWheelFF); 
    upperFlyController.setOutputRange(-1, 1, 0);
    upperFlyController.setSmartMotionMaxVelocity(Constants.Shooter.flyWheelMaxVel, Constants.Shooter.flyWheelMaxVelID);
    upperFlyController.setSmartMotionMaxAccel(Constants.Shooter.flyWheelMaxAccel, Constants.Shooter.flyWheelMaxAccelID);
    upperFlyController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.flyWheelAllowedError, Constants.Shooter.flyWheelAllowedErrorID);

    lowerFlyController.setFeedbackDevice(lowerFlyEncoder);
    lowerFlyController.setP(Constants.Shooter.flyWheelKP);
    lowerFlyController.setI(Constants.Shooter.flyWheelKI);
    lowerFlyController.setD(Constants.Shooter.flyWheelKD);
    lowerFlyController.setIZone(Constants.Shooter.flyWheelIZone); 
    lowerFlyController.setFF(Constants.Shooter.flyWheelFF); 
    lowerFlyController.setOutputRange(-1, 1, 0);
    lowerFlyController.setSmartMotionMaxVelocity(Constants.Shooter.flyWheelMaxVel, Constants.Shooter.flyWheelMaxVelID);
    lowerFlyController.setSmartMotionMaxAccel(Constants.Shooter.flyWheelMaxAccel, Constants.Shooter.flyWheelMaxAccelID);
    lowerFlyController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.flyWheelAllowedError, Constants.Shooter.flyWheelAllowedErrorID);
  }

  public void motorTurn(double flySpeed) {
    upperFlyMotor.set(flySpeed);
    lowerFlyMotor.set(flySpeed);
  }


  //Name might need to be changed
  public void setVelocity(double setVelocity) {
    upperFlyController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
    lowerFlyController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
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