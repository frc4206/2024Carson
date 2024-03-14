package frc.lib.util.spark.sparkConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class PIDConfig {
    double kP;
    double kI;
    double kIZone;
    double kD;
    double kFF;

    public PIDConfig(RelativeEncoder encoder, SparkPIDController pidController, double kP, double kI, double kIZone, double kD, double kFF){
        this.kP = kP;
        this.kI = kI;
        this.kIZone = kIZone;
        this.kD = kD;
        this.kFF = kFF;
    }

}
