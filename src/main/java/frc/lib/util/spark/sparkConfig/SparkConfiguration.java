package frc.lib.util.spark.sparkConfig;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * <p>
 * This class is a shortcut to initialize a
 * subsystem's motors, encoders, and PID controllers. 
 * 
 * <p>
 * This class should be declared before the subsystem constructor and then initialized with
 * the motor's appropriate constants in the subsystem constructor.
 * 
 * <p> 
 * DO NOT USE THIS CLASS IF THE SUBSYSTEM CONTAINS A SPARKPIDTUNER OBJECT.
*/
public class SparkConfiguration {

    /**
     * <p>
     * Creates an instance of a SparkConfiguration. This object is a shortcut to initialize a
     * subsystem's motor.
     * 
     * <p>
     * This object should be declared before the subsystem constructor and then initialized with
     * the motor's appropriate constants in the subsystem constructor.
     * 
     * <p> 
     * DO NOT USE THIS CLASS IF THE SUBSYSTEM CONTAINS A SPARKPIDTUNER OBJECT.
     * 
     * @param motor actual motor object
     * @param motorIsInverted whether the motor's output should be inverted
     * @param idleMode whether the motor should stop whenever no output is given or coast toward zero velocity
     * @param currentLimit supply current limit of the motor
     */
    public SparkConfiguration(CANSparkFlex motor, boolean motorIsInverted, IdleMode idleMode, int currentLimit){
        motor.restoreFactoryDefaults();
        motor.setInverted(motorIsInverted);
        motor.setIdleMode(idleMode);
        motor.setSmartCurrentLimit(currentLimit);
        motor.set(0);
    }

    /**
     * <p>
     * Creates an instance of a SparkConfiguration. This object is a shortcut to initialize a
     * subsystem's motors, encoders, and PID controllers. 
     * 
     * <p>
     * This object should be declared before the subsystem constructor and then initialized with
     * the motor's appropriate constants in the subsystem constructor.
     * 
     * <p> 
     * DO NOT USE THIS CLASS IF THE SUBSYSTEM CONTAINS A SPARKPIDTUNER OBJECT.
     * 
     * @param shouldRestore whether the motor should restore to factory default before configuring everything
     * @param shouldBurn whether the motor should burn the current configuration to flash 
     * @param motor actual motor object
     * @param motorIsInverted whether the motor's output should be inverted
     * @param idleMode whether the motor should stop whenever no output is given or coast toward zero velocity
     * @param currentLimit supply current limit of the motor
     * @param encoder actual encoder object
     * @param encoderIsInverted whether the encoder's position/velocity counter should be inverted
     * @param pidController actual PID controller object
     * @param motorkP P constant of the motor
     * @param motorkI I constant of the motor
     * @param motorkIZone IZone of the motor
     * @param motorkD D constant of the motor
     * @param motorkFF Feedforward constant of the motor
     * @param motorMaxVelo the max velocity the motor should spin
     * @param motorMaxAcc the max acceleration the motor should achieve
     * @param motorMaxError the max error threshold the motor should approach (if within threshold, stop)
     */
    public SparkConfiguration(boolean shouldRestore, boolean shouldBurn, CANSparkFlex motor, boolean motorIsInverted, IdleMode idleMode, int currentLimit, RelativeEncoder encoder, SparkPIDController pidController, double motorkP, double motorkI, double motorkIZone, double motorkD, double motorkFF, double motorMaxVelo, double motorMaxAcc, double motorMaxError){
        if (shouldRestore) {
            motor.restoreFactoryDefaults();
        }
        motor.setInverted(motorIsInverted);
        motor.setIdleMode(idleMode);
        motor.setSmartCurrentLimit(currentLimit);

        pidController.setFeedbackDevice(encoder);

        pidController.setP(motorkP);
        pidController.setI(motorkI);
        pidController.setIZone(motorkIZone);
        pidController.setD(motorkD);
        pidController.setFF(motorkFF);
        pidController.setOutputRange(-1, 1, 0);
        pidController.setSmartMotionMaxVelocity(motorMaxVelo, 0);
        pidController.setSmartMotionMaxAccel(motorMaxAcc, 0);
        pidController.setSmartMotionAllowedClosedLoopError(motorMaxError, 0);

        if (shouldBurn){
            motor.burnFlash();
        }
    }
}
