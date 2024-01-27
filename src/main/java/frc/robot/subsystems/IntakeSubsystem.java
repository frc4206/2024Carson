// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.invoke.ConstantBootstraps;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 



public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake_subsystem. */
  private CANSparkFlex intakeMotor = new CANSparkFlex(Constants.Intake.IntakeDriveMotorID, MotorType.kBrushless);
  private DigitalInput beambreak = new DigitalInput(Constants.Intake.IntkeBeamBreakDIO); 

  public boolean beambreakValue = beambreak.get();
  

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      beambreakValue = !beambreak.get();
      SmartDashboard.putBoolean("Beambreak Activated", beambreakValue); 
      intakeMotor.setInverted(true);
      
    }


    public boolean state() {
      return beambreak.get(); 
    }

    public void GoUntilBeamBreak(double setSpeed) {
      if (beambreakValue == false) {
        intakeMotor.set(setSpeed); 
      } else {
        intakeMotor.set(0); 
      }
    }

    public void IntakeGo(double setSpeed) {
      intakeMotor.set(setSpeed);
    }

}
