// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class LEDs extends SubsystemBase {
	public enum State {
		ALIGNED,
		NOTALIGNED,
		INTAKING,
		PIECED,
		ATVELO
	}

  protected final CANdle candle;
  public State state = State.NOTALIGNED;

	public LEDs() {
		candle = new CANdle(Constants.LEDS.candleID, "rio");

		CANdleConfiguration config = new CANdleConfiguration();
		config.v5Enabled = false;
		config.disableWhenLOS = true;
		config.statusLedOffWhenActive = false;
		config.vBatOutputMode = CANdle.VBatOutputMode.Off;

		candle.configAllSettings(config);

		SmartDashboard.putBoolean("PIECEREADY", false);
 	}
      
	private StrobeAnimation solidLEDS(int red, int green, int blue, int white, int numLEDs, int ledStart){
		return new StrobeAnimation(red, green, blue, white, 1, numLEDs, ledStart);
	}
	
	private StrobeAnimation flashLEDS(int red, int green, int blue, int white, int numLEDs, int ledStart){
		return new StrobeAnimation(red, green, blue, white, 0.1, numLEDs, ledStart);
	}

	private void aligned(){
		if (GlobalVariables.alliance == DriverStation.Alliance.Red) {
			candle.animate(solidLEDS(255, 0, 0, 0, Constants.LEDS.numCandleLEDs, Constants.LEDS.ledCandleBegin));
			candle.animate(solidLEDS(0, 255, 0, 0, Constants.LEDS.numShooterLEDs, Constants.LEDS.ledShooterBegin));
			candle.animate(solidLEDS(255, 0, 0, 0, Constants.LEDS.numFrontLEDs, Constants.LEDS.ledFrontBegin));	
		} else if (GlobalVariables.alliance == DriverStation.Alliance.Blue){
			candle.animate(solidLEDS(0, 0, 255, 0, Constants.LEDS.numCandleLEDs, Constants.LEDS.ledCandleBegin));
			candle.animate(solidLEDS(0, 255, 0, 0, Constants.LEDS.numShooterLEDs, Constants.LEDS.ledShooterBegin));
			candle.animate(solidLEDS(0, 0, 255, 0, Constants.LEDS.numFrontLEDs, Constants.LEDS.ledFrontBegin));	
		}
	}

	private void notAligned(){
		if (GlobalVariables.alliance == DriverStation.Alliance.Red) {
			candle.animate(solidLEDS(255, 0, 0, 0, Constants.LEDS.numCandleLEDs, Constants.LEDS.ledCandleBegin));
			candle.animate(solidLEDS(222, 0, 0, 0, Constants.LEDS.numShooterLEDs, Constants.LEDS.ledShooterBegin));
			candle.animate(solidLEDS(255, 0, 0, 0, Constants.LEDS.numFrontLEDs, Constants.LEDS.ledFrontBegin));	
		} else if (GlobalVariables.alliance == DriverStation.Alliance.Blue){
			candle.animate(solidLEDS(0, 0, 255, 0, Constants.LEDS.numCandleLEDs, Constants.LEDS.ledCandleBegin));
			candle.animate(solidLEDS(255, 0, 0, 0, Constants.LEDS.numShooterLEDs, Constants.LEDS.ledShooterBegin));
			candle.animate(solidLEDS(0, 0, 255, 0, Constants.LEDS.numFrontLEDs, Constants.LEDS.ledFrontBegin));	
		}	
	}

	private void intaking(){
		candle.animate(solidLEDS(255, 255, 255, 0, Constants.LEDS.numShooterLEDs, Constants.LEDS.ledShooterBegin));
	}

	private void ready(){
		candle.animate(flashLEDS(0, 255, 0, 0, Constants.LEDS.numShooterLEDs, Constants.LEDS.ledShooterBegin));
	}

	private void atVelo(){
		candle.animate(solidLEDS(0, 0, 255, 0, Constants.LEDS.numShooterLEDs, Constants.LEDS.ledShooterBegin));
	}
	
	private void alliance() {
		if (GlobalVariables.alliance == DriverStation.Alliance.Red) {
			candle.animate(solidLEDS(255, 0, 0, 0, Constants.LEDS.numLEDs, Constants.LEDS.ledStart));
		} else if (GlobalVariables.alliance == DriverStation.Alliance.Blue){
			candle.animate(solidLEDS(0, 0, 255, 0, Constants.LEDS.numLEDs, Constants.LEDS.ledStart));
		}
	}


	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) {
			// if at starting pose
				// state = State.ALIGNED;
			// else
				state = State.NOTALIGNED;
		} else {
			if (Math.abs(GlobalVariables.Intake.intakeDuty) > 0){
				state = State.INTAKING;
			}
			if (GlobalVariables.Conveyor.beamBroken){
				state = State.PIECED;
			}
			if (GlobalVariables.Flywheel.topVelo > 6250 && GlobalVariables.Flywheel.bottomVelo > 6250){
				state = State.ATVELO;
			}
			if (!(Math.abs(GlobalVariables.Intake.intakeDuty) > 0) && !(GlobalVariables.Conveyor.beamBroken) && (GlobalVariables.Flywheel.topVelo > 6250 && GlobalVariables.Flywheel.bottomVelo > 6250)){
				state = State.NOTALIGNED;
			}
		}

		switch (state) {
			case ALIGNED:
				aligned();
				break;
			case NOTALIGNED:
				notAligned();
				break;
			case INTAKING:
				intaking();
				break;
			case PIECED:
				ready();
				break;
			case ATVELO:
				atVelo();
				break;
			default:
				alliance();
				break;
		} 
	}
}