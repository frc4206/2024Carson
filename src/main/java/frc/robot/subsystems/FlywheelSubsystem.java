// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

//THIS SUBSYSTEM CONTAINS THE FLYWHEELS, INTAKE, AND PIVOT

public class FlywheelSubsystem extends SubsystemBase {

  public CANSparkFlex upperFlyMotor = new CANSparkFlex(Constants.Shooter.ShooterLeadMotorID, MotorType.kBrushless); // TMNTBC!
  public CANSparkFlex lowerFlyMotor = new CANSparkFlex(Constants.Shooter.ShooterFollowerID, MotorType.kBrushless);
  //public CANSparkFlex pivotMotor = new CANSparkFlex(3, MotorType.kBrushless);
  //public CANSparkFlex intakeMotor = new CANSparkFlex(4, MotorType.kBrushless);

  public DigitalInput shooterBeamBreak = new DigitalInput(Constants.Shooter.ShooterBeamBreak);

  public SparkPIDController flyController;
  public SparkPIDController pivotController;

  public RelativeEncoder relFlyEnc;

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem() {
    upperFlyMotor.restoreFactoryDefaults();
    lowerFlyMotor.restoreFactoryDefaults();
    //pivotMotor.restoreFactoryDefaults();
    //intakeMotor.restoreFactoryDefaults();
//
    //pivotMotor.setIdleMode(IdleMode.kBrake);//might be bad IDK

    flyController = upperFlyMotor.getPIDController();

    relFlyEnc = upperFlyMotor.getEncoder();

    lowerFlyMotor.follow(upperFlyMotor);

    flyController.setFeedbackDevice(relFlyEnc);
    flyController.setP(Constants.Shooter.flyWheelKP);
    flyController.setI(Constants.Shooter.flyWheelKI);
    flyController.setD(Constants.Shooter.flyWheelKD);

    flyController.setIZone(Constants.Shooter.flyWheelIZone); 
    flyController.setFF(Constants.Shooter.flyWheelFF); 
    flyController.setSmartMotionMaxVelocity(Constants.Shooter.flyWheelMaxVel, Constants.Shooter.flyWheelMaxVelID);
    flyController.setSmartMotionMinOutputVelocity(Constants.Shooter.flyWheelMinVel, Constants.Shooter.flyWheelMinVelID);
    flyController.setSmartMotionMaxAccel(Constants.Shooter.flyWheelMaxAccel, Constants.Shooter.flyWheelMaxAccelID);
    flyController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.flyWheelAllowedError, Constants.Shooter.flyWheelAllowedErrorID);
  }

  public void motorTurn(double flySpeed) {}

  public void motorPivot(double pivotSpeed) {
   // pivotMotor.set(pivotSpeed);
  }

  public void IntakeMotor(double inSpeed) {
 //   intakeMotor.set(inSpeed);
  }

  //Name might need to be changed
  public void setVelocity(double setVelocity) {
    flyController.setReference(setVelocity, CANSparkFlex.ControlType.kVelocity);
  }

  /*public void goToPos(double setPos) {
    flyController.setReference(setPos, CANSparkFlex.ControlType.kPosition);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Beam Break Activated?", shooterBeamBreak.get());
  }
}
 