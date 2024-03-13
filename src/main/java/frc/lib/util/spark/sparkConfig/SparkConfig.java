package frc.lib.util.spark.sparkConfig;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class SparkConfig {

    MotorConfig motorConfig;
    PIDConfig pidConfig;
    FeedbackConfig outputConfig;
    CANSparkFlex motor;
    RelativeEncoder encoder;
    SparkPIDController pidController;

    public SparkConfig(MotorConfig motorConfig, PIDConfig pidConfig, FeedbackConfig outputConfig, CANSparkFlex motor, RelativeEncoder encoder, SparkPIDController pidController){
        this.motorConfig = motorConfig;
        this.pidConfig = pidConfig;
        this.outputConfig = outputConfig;
        this.motor = motor;
        this.encoder = encoder;
        this.pidController = pidController;
    }

    public void applyConfig(){
        motor = new CANSparkFlex(motorConfig.canID, motorConfig.motorType);
        encoder = motor.getEncoder();
        pidController = motor.getPIDController();

        
    }
}
