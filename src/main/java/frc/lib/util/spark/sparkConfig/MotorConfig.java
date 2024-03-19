package frc.lib.util.spark.sparkConfig;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Utility class for initializing Motor configurations of a CANSparkFlex controller
 */
public class MotorConfig {
    int canID;
    MotorType motorType;
    boolean motorIsInverted;
    IdleMode idleMode;
    int currentLimit;
    double closedLoopRampRate;

    /**
     * Creates an instance of MotorConfig, a
     * Utility class for initializing Motor configurations of a CANSparkFlex controller
     * 
     * @param canID actual can ID of CANSparkFlex
     * @param motorIsInverted whether the motor's output should be inverted
     * @param idleMode whether the motor should brake or coast to zero
     */
    public MotorConfig(int canID, boolean motorIsInverted, IdleMode idleMode){
        this.canID = canID;
        this.motorIsInverted = motorIsInverted;
        this.idleMode = idleMode;
        this.currentLimit = 0;
        this.closedLoopRampRate = 0;
    }

    /**
     * Creates an instance of MotorConfig, a
     * Utility class for initializing Motor configurations of a CANSparkFlex controller
     * 
     * @param canID actual can ID of CANSparkFlex
     * @param motorIsInverted whether the motor's output should be inverted
     * @param idleMode whether the motor should brake or coast to zero
     * @param currentLimit how much supply current the motor should use at any time
     */
    public MotorConfig(int canID, boolean motorIsInverted, IdleMode idleMode, int currentLimit){
        this.canID = canID;
        this.motorIsInverted = motorIsInverted;
        this.idleMode = idleMode;
        this.currentLimit = currentLimit;
        this.closedLoopRampRate = 0;
    }

    /**
     * Creates an instance of MotorConfig, a
     * Utility class for initializing Motor configurations of a CANSparkFlex controller
     * 
     * @param canID actual can ID of CANSparkFlex
     * @param motorIsInverted whether the motor's output should be inverted
     * @param idleMode whether the motor should brake or coast to zero
     * @param currentLimit how much supply current the motor should use at any time
     * @param closedLoopRampRate how much the motor can accelerate from 0 to full duty
     */
    public MotorConfig(int canID, boolean motorIsInverted, IdleMode idleMode, int currentLimit, double closedLoopRampRate){
        this.canID = canID;
        this.motorIsInverted = motorIsInverted;
        this.idleMode = idleMode;
        this.currentLimit = currentLimit;
        this.closedLoopRampRate = closedLoopRampRate;
    }
}
