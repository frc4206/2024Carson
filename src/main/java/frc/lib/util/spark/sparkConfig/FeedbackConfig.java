package frc.lib.util.spark.sparkConfig;

public class FeedbackConfig {
    double minDuty;
    double maxDuty;
    double maxVelo;
    double maxAcc;
    double maxError;

    public FeedbackConfig(double minDuty, double maxDuty, double maxVelo, double maxAcc, double maxError){
        this.minDuty = minDuty;
        this.maxDuty = maxDuty;
        this.maxVelo = maxVelo;
        this.maxAcc = maxAcc;
        this.maxError = maxError;
    }
}
