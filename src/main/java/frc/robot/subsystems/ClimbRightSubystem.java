// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbRightSubystem extends SubsystemBase {
	private CANSparkFlex climberRightLead = new CANSparkFlex(Constants.Climber.climberRightLeadID, MotorType.kBrushless);
  private PWM servoRight = new PWM(1);

  public ClimbRightSubystem() {
    climberRightLead.restoreFactoryDefaults();
    climberRightLead.setIdleMode(IdleMode.kBrake);
    climberRightLead.setInverted(false);
    climberRightLead.burnFlash();
  }

  public void climbStop(){
    climberRightLead.set(0);
  }

  public void climberUp(){
    climberRightLead.set(0.2);
  }

  public void climberDown(){
    climberRightLead.set(-0.2);
  }

  public void setPosition(double pos){
    servoRight.setPosition(pos);
  }

  @Override
  public void periodic() {

  }
}
