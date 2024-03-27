package frc.lib.util.spark;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

/**
 * <p>
 * This interface is a shortcut for simple CANSparkFlex motor methods
 * such as setting a motor's duty cycle, resetting the motor's encoder
 * position, telling the motor's PID controller to go to a specific position
 * and telling the motor's PID controller to go to a specific velocity.
 */
public interface SparkDefaultMethods {

    /**
     * <p>
     * Sets a motor's duty cycle.
     * 
     * @param motor actual motor object
     * @param motorSpeed desired duty cycle of the motor (range: [-1, 1])
     */
    public default void motorToDuty(CANSparkFlex motor, double motorSpeed){
        motor.set(motorSpeed);
    }

    /**
     * <p>
     * Sets a motor's duty cycle.
     * 
     * @param motor actual motor object
     * @param motorSpeed desired duty cycle of the motor (range: [-1, 1])
     */
    public default void motorToDuty(CANSparkMax motor, double motorSpeed){
        motor.set(motorSpeed);
    }

    /**
     * <p>
     * Resets a motor's relative position.
     * 
     * @param encoder actual encoder object
     */
    public default void resetEncoder(RelativeEncoder encoder){
        encoder.setPosition(0);
    }

    /**
     * <p>
     * Resets a motor's relative position.
     * 
     * @param encoder actual encoder object
     * @param position position to set the encoder to
     */
    public default void resetEncoder(RelativeEncoder encoder, double position){
        encoder.setPosition(position);
    }

    /**
     * <p>
     * Communicates a desired position to the motor's PID controller and 
     * travels toward the desired position
     * 
     * @param pidController actual PID controller object
     * @param setPosition desired position for the motor to travel towards
     */
    public default void motorToPosition(SparkPIDController pidController, double setPosition){
        pidController.setReference(setPosition, ControlType.kPosition);
    }

    /**
     * <p>
     * Communicates a desired velocity to the motor's PID controller and 
     * approaches the desired velocity
     * 
     * @param pidController actual PID controller object
     * @param setVelocity desired velocity for the motor to approach
     */
    public default void motorToVelocity(SparkPIDController pidController, double setVelocity){
        pidController.setReference(setVelocity, ControlType.kVelocity);
    }
}