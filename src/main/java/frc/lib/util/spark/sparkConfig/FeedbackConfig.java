package frc.lib.util.spark.sparkConfig;

/**
 * Utility class for initializing Feedback configurations of a CANSparkFlex controller.
 */
public class FeedbackConfig {
    double minDuty;
    double maxDuty;
    double maxVelo;
    double maxAcc;
    double maxError;

    /**
     * Creates an instance of FeedbackConfig, a
     * Utility class for initializing Feedback configurations of a CANSparkFlex controller.
     * @param minDuty minimum duty the motor will use in a PID
     * @param maxDuty maximum duty the motor will use in a PID
     * @param maxVelo maximum velocity the motor will use in a PID
     * @param maxAcc maximum acceleration the motor will have in a PID
     * @param maxError maximum error the motor will approach in a PID
     */
    public FeedbackConfig(double minDuty, double maxDuty, double maxVelo, double maxAcc, double maxError){
        this.minDuty = minDuty;
        this.maxDuty = maxDuty;
        this.maxVelo = maxVelo;
        this.maxAcc = maxAcc;
        this.maxError = maxError;
    }
}
