package frc.lib.util.spark.sparkConfig;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Utility class for initializing a CANSparkFlex controller.
 */
public class ControllerConfig {
    int canID;
    CANSparkFlex motor;
    RelativeEncoder encoder;
    SparkPIDController pidController;

    /**
     * Creates an instance of ControllerConfig, a
     * Utility class for initializing a CANSparkFlex controller.
     * @param motor actual motor object
     * @param encoder actual encoder object 
     * @param pidController actual pidController object
     */
    public ControllerConfig(int canID, CANSparkFlex motor){
        this.canID = canID;
        this.motor = motor;
        this.encoder = null;
        this.pidController = null;
    }

    /**
     * Creates an instance of ControllerConfig, a
     * Utility class for initializing a CANSparkFlex controller.
     * @param motor actual motor object
     * @param encoder actual encoder object 
     * @param pidController actual pidController object
     */
    public ControllerConfig(int canID, CANSparkFlex motor, RelativeEncoder encoder, SparkPIDController pidController){
        this.canID = canID;
        this.motor = motor;
        this.encoder = encoder;
        this.pidController = pidController;
    }

    public CANSparkFlex getMotor(){
        motor = new CANSparkFlex(canID, MotorType.kBrushless);
        return motor;
    }

    public RelativeEncoder getEncoder(){
        encoder = motor.getEncoder();
        return encoder;
    }

    public SparkPIDController getPIDController(){
        pidController = motor.getPIDController();
        return pidController;
    }
}
