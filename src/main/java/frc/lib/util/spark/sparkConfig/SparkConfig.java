package frc.lib.util.spark.sparkConfig;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * Utility class for initializing all configurations of a CANSparkFlex controller.
 */
public class SparkConfig {
    MotorConfig motorConfig;
    PIDConfig pidConfig;
    FeedbackConfig feedbackConfig;
    CANSparkFlex motor;
    RelativeEncoder encoder;
    SparkPIDController pidController;
    boolean shouldRestore;
    boolean shouldBurn;

    /**
     * Creates an instance of SparkConfig, a
     * Utility class for initializing all configurations of a CANSparkFlex controller.
     * 
     * @param motorConfig actual motorconfig object
     * @param pidConfig actual pidconfig object
     * @param feedbackConfig actual feedbackConfig object
     * @param motor actual motor object
     * @param encoder actual encoder object
     * @param pidController actual PIDController object
     * @param shouldRestore whether the motor should restore to factory default upon intialization
     * @param shouldBurn whether the motor should burn the current configuration to flash
     */
    public SparkConfig(MotorConfig motorConfig, PIDConfig pidConfig, FeedbackConfig feedbackConfig, CANSparkFlex motor, RelativeEncoder encoder, SparkPIDController pidController, boolean shouldRestore, boolean shouldBurn){
        this.motorConfig = motorConfig;
        this.pidConfig = pidConfig;
        this.feedbackConfig = feedbackConfig;
        this.motor = motor;
        this.encoder = encoder;
        this.pidController = pidController;
        this.shouldRestore = shouldRestore;
        this.shouldBurn = shouldBurn;
    }

    /**
     * Applies all configurations to the motor controller
     */
    public void applyConfig(){
        motor = new CANSparkFlex(motorConfig.canID, motorConfig.motorType);
        encoder = motor.getEncoder();
        pidController = motor.getPIDController();

        if (shouldRestore){
            motor.restoreFactoryDefaults();
        }
        motor.setInverted(motorConfig.motorIsInverted);
        motor.setIdleMode(motorConfig.idleMode);
        motor.setSmartCurrentLimit(motorConfig.currentLimit);

        pidController.setFeedbackDevice(encoder);

        pidController.setP(pidConfig.kP);
        pidController.setI(pidConfig.kI);
        pidController.setIZone(pidConfig.kIZone);
        pidController.setD(pidConfig.kD);
        pidController.setFF(pidConfig.kFF);
        pidController.setOutputRange(feedbackConfig.minDuty, feedbackConfig.maxDuty);
        pidController.setSmartMotionMaxVelocity(feedbackConfig.maxVelo, 0);
        pidController.setSmartMotionMaxAccel(feedbackConfig.maxAcc, 0);
        pidController.setSmartMotionAllowedClosedLoopError(feedbackConfig.maxError, 0);

        if (shouldBurn){
            motor.burnFlash();
        }
    }
}