// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.spark.SparkDefaultMethods;
import frc.lib.util.spark.sparkConfig.ControllerConfig;
import frc.lib.util.spark.sparkConfig.FeedbackConfig;
import frc.lib.util.spark.sparkConfig.MotorConfig;
import frc.lib.util.spark.sparkConfig.PIDConfig;
import frc.lib.util.spark.sparkConfig.SparkConfig;
import frc.robot.Constants;

public class AmpBarSubsystem extends SubsystemBase implements SparkDefaultMethods {
  private CANSparkFlex ampBarMotor;
  private RelativeEncoder ampBarEncoder;
  private SparkPIDController ampBarPIDController;
  SparkConfig ampBarConfig;
  ControllerConfig controllerConfig;

  public AmpBarSubsystem() {
    controllerConfig = new ControllerConfig(Constants.AmpBar.ampBarMotorID, ampBarMotor, ampBarEncoder, ampBarPIDController);
    ampBarMotor = controllerConfig.getMotor();
    ampBarEncoder = controllerConfig.getEncoder();
    ampBarPIDController = controllerConfig.getPIDController();

    ampBarConfig = new SparkConfig(
      new FeedbackConfig(-1, 1, Constants.AmpBar.ampBarMaxVelo, Constants.AmpBar.ampBarMaxAcc, Constants.AmpBar.ampBarMaxError), 
      new MotorConfig(Constants.AmpBar.ampBarMotorID, true, IdleMode.kBrake), 
      new PIDConfig(Constants.AmpBar.ampBarkP, Constants.AmpBar.ampBarkI, Constants.AmpBar.ampBarkIZone, Constants.AmpBar.ampBarkD, Constants.AmpBar.ampBarkFF), 
      ampBarMotor,
      ampBarEncoder, 
      ampBarPIDController, 
      true, 
      true
    );
    ampBarConfig.applyConfig();
  }

  public boolean ampBarWithinRange(double desiredPosition){
    return Math.abs(ampBarEncoder.getPosition() - desiredPosition) < Constants.AmpBar.ampBarMaxError;
  }

  public void resetAmpBar(){
    resetMotor(ampBarEncoder);
  }

  public void runAmpBar(double speed){
    setMotorSpeed(ampBarMotor, speed);
  }

  public void setPosition(double desiredPosition){
    motorGoToPosition(ampBarPIDController, desiredPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ampBarPosition", ampBarEncoder.getPosition());
  }
}
