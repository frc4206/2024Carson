package frc.lib.util.spark.sparkConfig;

/**
 * Utility class for initializing PID configurations of a CANSparkFlex controller
 */
public class PIDConfig {
    double kP;
    double kI;
    double kIZone;
    double kD;
    double kFF;

    /**
     * Creates an instance of PIDConfig, a
     * Utility class for initializing Motor configurations of a CANSparkFlex controller
     * 
     * @param kP P constant of the PID
     * @param kI I constant of the PID
     * @param kIZone I Zone constant of the PID
     * @param kD D constant of the PID
     * @param kFF FF constant of the PID
     */
    public PIDConfig(double kP, double kI, double kIZone, double kD, double kFF){
        this.kP = kP;
        this.kI = kI;
        this.kIZone = kIZone;
        this.kD = kD;
        this.kFF = kFF;
    }
}
