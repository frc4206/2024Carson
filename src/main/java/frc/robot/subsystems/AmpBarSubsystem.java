// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class AmpBarSubsystem extends SubsystemBase implements SparkDefaultMethods {
  private CANSparkFlex ampBarMotor = new CANSparkFlex(Constants.AmpBar.ampBarMotorID, MotorType.kBrushless);
  private RelativeEncoder ampBarEncoder = ampBarMotor.getEncoder();
  private SparkPIDController ampBarPIDController = ampBarMotor.getPIDController();
  SparkConfig ampBarConfig;

  private DigitalInput ampLimitSwitch = new DigitalInput(Constants.AmpBar.ampLimitSwitchID);
  private DigitalInput hallZeroLimitSwitch = new DigitalInput(Constants.AmpBar.hallZeroLimitSwitchID);
  private DigitalInput zeroLimitSwitch = new DigitalInput(Constants.AmpBar.zeroLimitSwitchID);

  public AmpBarSubsystem() {
    ampBarConfig = Constants.AmpBar.ampBarConfig;
    ampBarConfig.configureController(ampBarMotor, ampBarEncoder, ampBarPIDController);
    ampBarConfig.applyAllConfigurations();
  }

  public boolean ampBarAtPosition(double desiredPosition){
    return Math.abs(ampBarEncoder.getPosition() - desiredPosition) < Constants.AmpBar.ampBarMaxError;
  }

  public void ampBarToDuty(double speed){
    motorToDuty(ampBarMotor, speed);
  }

  public void ampBarToPosition(double desiredPosition){
    motorToPosition(ampBarPIDController, desiredPosition);
  }

  @Override
  public void periodic() {
    GlobalVariables.AmpBar.ampBarPosition = ampBarEncoder.getPosition();
    GlobalVariables.AmpBar.ampBarAtLimitSwitch = !ampLimitSwitch.get() || !zeroLimitSwitch.get() || !hallZeroLimitSwitch.get();
    GlobalVariables.AmpBar.ampBarAtAmp = !ampLimitSwitch.get();
    GlobalVariables.AmpBar.ampBarAtZero = !zeroLimitSwitch.get() || !hallZeroLimitSwitch.get();


    SmartDashboard.putNumber("ampBarPosition", GlobalVariables.AmpBar.ampBarPosition);
    SmartDashboard.putBoolean("ampBarAtLimitSwitch", GlobalVariables.AmpBar.ampBarAtLimitSwitch);
    SmartDashboard.putBoolean("ampBarAtZero", GlobalVariables.AmpBar.ampBarAtZero);
    SmartDashboard.putBoolean("ampBarAtAmp", GlobalVariables.AmpBar.ampBarAtAmp);
    SmartDashboard.putNumber("ampBarCurrent", ampBarMotor.getOutputCurrent());
  }
}
