// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public DigitalInput beamBreak = new DigitalInput(1);

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
    flyController.setP(0.00029);
    flyController.setI(7e-7);
    flyController.setD(0.0);
    flyController.setIZone(0);
    flyController.setFF(0.0);
    flyController.setSmartMotionMaxVelocity(1, 0);
    flyController.setSmartMotionMinOutputVelocity(-1, 0);
    flyController.setSmartMotionMaxAccel(100, 0);
    flyController.setSmartMotionAllowedClosedLoopError(5, 0);
//

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
    SmartDashboard.putBoolean("Beam Break Activated?", beamBreak.get());
  }
}
 