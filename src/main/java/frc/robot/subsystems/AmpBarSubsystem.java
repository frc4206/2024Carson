// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class AmpBarSubsystem extends SubsystemBase implements SparkDefaultMethods {
  private CANSparkMax ampBarMotor = new CANSparkMax(Constants.AmpBar.ampBarMotorID, MotorType.kBrushed);
  private DutyCycleEncoder ampBarEncoder = new DutyCycleEncoder(2);
  SparkConfig ampBarConfig;

  private DigitalInput ampLimitSwitch = new DigitalInput(Constants.AmpBar.ampLimitSwitchID);
  private DigitalInput zeroLimitSwitch = new DigitalInput(Constants.AmpBar.zeroLimitSwitchID);

  double error = 0;
  double prevError = 0;
  double integratedError = 0;
  double derivative = 0;
  double currentPosition = 0;
  double output = 0;

  public AmpBarSubsystem() {}

  public double map(double val, double inMin, double inMax, double outMin, double outMax) {
		return ((val - inMin) * (outMax - outMin) / (inMax - inMin)) + outMin;
	}

  public boolean ampBarAtPosition(double desiredPosition){
    return Math.abs(ampBarEncoder.getAbsolutePosition() - desiredPosition) < Constants.AmpBar.ampBarMaxError;
  }

  public void ampBarToDuty(double speed){
    motorToDuty(ampBarMotor, speed);
  }

  public void ampBarToPosition(double desiredPosition){
    currentPosition = ampBarEncoder.getAbsolutePosition();
    error = desiredPosition - currentPosition;
    integratedError += error;
    derivative = error - prevError;

    output = (Constants.AmpBar.ampBarkP * error) + (Constants.AmpBar.ampBarkI * integratedError) + (Constants.AmpBar.ampBarkD * derivative);
    if (output > 1){
      output = 1;
    } else if (output < -1){
      output = -1;
    }
    prevError = error;

    ampBarMotor.set(output);
  }

  @Override
  public void periodic() {
    GlobalVariables.AmpBar.ampBarPosition = ampBarEncoder.getAbsolutePosition();
    GlobalVariables.AmpBar.ampBarAtLimitSwitch = !ampLimitSwitch.get() || !zeroLimitSwitch.get();
    GlobalVariables.AmpBar.ampBarAtAmp = !ampLimitSwitch.get();
    GlobalVariables.AmpBar.ampBarAtZero = !zeroLimitSwitch.get();


    SmartDashboard.putNumber("ampBarPosition", GlobalVariables.AmpBar.ampBarPosition);
    SmartDashboard.putBoolean("ampBarAtLimitSwitch", GlobalVariables.AmpBar.ampBarAtLimitSwitch);
    SmartDashboard.putBoolean("ampBarAtZero", GlobalVariables.AmpBar.ampBarAtZero);
    SmartDashboard.putBoolean("ampBarAtAmp", GlobalVariables.AmpBar.ampBarAtAmp);
    SmartDashboard.putNumber("ampBarCurrent", ampBarMotor.getOutputCurrent());
  }
}
