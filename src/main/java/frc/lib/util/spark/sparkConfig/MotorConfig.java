package frc.lib.util.spark.sparkConfig;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorConfig {
    int canID;
    MotorType motorType;
    boolean motorIsInverted;
    IdleMode idleMode;
    int currentLimit;

    public MotorConfig(int canID, boolean motorIsInverted, IdleMode idleMode, int currentLimit){
        this.canID = canID;
        this.motorIsInverted = motorIsInverted;
        this.idleMode = idleMode;
        this.currentLimit = currentLimit;
    }
}
