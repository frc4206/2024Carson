// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class LEDSubsystem extends SubsystemBase {
	public enum State {
		ALIGNED,
		NOTALIGNED,
		INTAKING,
		READY,
		AMPREADY
	}

  protected final CANdle candle;
  private State state = State.NOTALIGNED;

	public LEDSubsystem() {
		candle = new CANdle(Constants.LEDS.candleID, Constants.Canivore1);

		CANdleConfiguration config = new CANdleConfiguration();
		config.v5Enabled = false;
		config.disableWhenLOS = true;
		config.statusLedOffWhenActive = false;
		config.vBatOutputMode = CANdle.VBatOutputMode.Off;

	candle.configAllSettings(config);
 	}
      
  private StrobeAnimation solidLEDS(int red, int green, int blue, int white){
    return new StrobeAnimation(red, green, blue, white, 1, Constants.LEDS.numLEDs, Constants.LEDS.ledStartOffset);
  }
  
  private StrobeAnimation flashLEDS(int red, int green, int blue, int white){
    return new StrobeAnimation(red, green, blue, white, 0.25, Constants.LEDS.numLEDs, Constants.LEDS.ledStartOffset);
  }

	private void halfGreen(){
		candle.animate(new StrobeAnimation(0, 255, 0, 0, 0, Constants.LEDS.numLEDs/2, Constants.LEDS.ledStartOffset));
	}

	private void halfRed(){
		candle.animate(new StrobeAnimation(255, 0, 0, 0, 0, Constants.LEDS.numLEDs/2, Constants.LEDS.ledStartOffset));
	}

	private void halfAlliance(){
		candle.animate(new StrobeAnimation(0, 255, 0, 0, 0, Constants.LEDS.numLEDs, Constants.LEDS.numLEDs/2));
	}

	private void intaking(){
		candle.animate(solidLEDS(0, 0, 0, 255));
	}

	private void ready(){
		candle.animate(flashLEDS(0, 255, 0, 0));
	}

	private void ampReady(){
		candle.animate(flashLEDS(255, 191, 0, 0));
	}

	private void alliance() {
		if (GlobalVariables.alliance == DriverStation.Alliance.Red) {
			candle.animate(solidLEDS(255, 0, 0, 0));
		} else if (GlobalVariables.alliance == DriverStation.Alliance.Blue){
			candle.animate(solidLEDS(0, 0, 255, 0));
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
			if (GlobalVariables.intakingPiece) {
				state = State.INTAKING;
			} 
			if (GlobalVariables.pieceReady) {
				state = State.READY;
			}
			if (GlobalVariables.ampReady) {
				state = State.AMPREADY;
			}
		}

		switch (state) {
			case ALIGNED:
				halfGreen();
				halfAlliance();
			case NOTALIGNED:
				halfRed();
				halfAlliance();
			case INTAKING:
				intaking();
			case READY:
				ready();
			case AMPREADY:
				ampReady();
			default:
				alliance();
				break;
		} 
	}
}