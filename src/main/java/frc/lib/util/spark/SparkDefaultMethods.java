package frc.lib.util.spark;

import com.revrobotics.CANSparkFlex;
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
    public default void setMotorSpeed(CANSparkFlex motor, double motorSpeed){
        motor.set(motorSpeed);
    }

    /**
     * <p>
     * Resets a motor controller's position.
     * 
     * @param encoder actual encoder object
     */
    public default void resetMotor(RelativeEncoder encoder){
        encoder.setPosition(0);
    }

    /**
     * <p>
     * Communicates a desired position to the motor's PID controller and 
     * moves toward the desired position
     * 
     * @param pidController actual PID controller object
     * @param setPosition desired position for the motor to travel towards
     */
    public default void motorGoToPosition(SparkPIDController pidController, double setPosition){
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
    public default void motorGoToVelocity(SparkPIDController pidController, double setVelocity){
        pidController.setReference(setVelocity, ControlType.kVelocity);
    }
}