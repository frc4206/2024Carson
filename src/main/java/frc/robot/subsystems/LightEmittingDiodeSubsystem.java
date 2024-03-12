// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class LightEmittingDiodeSubsystem extends SubsystemBase {

    public AddressableLED leds; 
    public AddressableLEDBuffer buffer;

    public int pwm_port;
    public int number_of_leds;

    // Bit timings for WS2812B !!! 
    // all times are in nanoseconds (0.000_000_001th of a second)
    public static final int zero_high = 400;
    public static final int zero_low = 850;
    public static final int one_high = 800;
    public static final int one_low = 450;

    public LightEmittingDiodeSubsystem(int pwm_port, int number_of_leds)
    {
        this.pwm_port = pwm_port;
        this.number_of_leds = number_of_leds;

        // new LED object with timing for driver
        this.leds = new AddressableLED(pwm_port); // <-- number of PWM port  
        this.leds.setBitTiming(zero_high, zero_low, one_high, one_low);

        // setup LED buffer
        this.buffer = new AddressableLEDBuffer(number_of_leds);
        leds.setLength(number_of_leds);

        // default setup and start
        this.setLEDs(50, 50, 50);
        leds.start();
    }

    public void setLEDs(int red, int green, int blue)
    {
        for(int i = 0; i < this.buffer.getLength(); i++)
        {
            this.buffer.setRGB(i, red, green, blue);
        }
        this.leds.setData(buffer);
    }

}

