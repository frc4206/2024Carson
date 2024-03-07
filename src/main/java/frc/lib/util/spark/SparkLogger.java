package frc.lib.util.spark;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <p>
 * This class is a shortcut to logging all important motor information
 * including AppliedOutput (dutycycle units), BusVoltage (volts), MotorTemperature (celsius),
 * OutputCurrent (amps), Position (motorCounts), and Velocity (rpm)
 */
public class SparkLogger {
    private String motorName;

    /**
     * Creates an instance of SparkLogger. This object is a shortcut to log all important motor information
     * including AppliedOutput (dutycycle units), BusVoltage (volts), MotorTemperature (celsius),
     * OutputCurrent (amps), Position (motorCounts), and Velocity (rpm)
     * 
     * @param motorName name of the motor (conveyorMotor => conveyor is the name to enter in the parameter)
     */
    public SparkLogger(String motorName){
        this.motorName = motorName;
    }

    /**
     * <p>
     * Log important Motor Info including AppliedOutput (dutycycle units), BusVoltage (volts), MotorTemperature (celsius),
     * OutputCurrent (amps), Position (motorCounts), and Velocity (rpm)
     * 
     * <p>
     * Call this function in the periodic function of your subsystem for periodic logging.
     * 
     * <p>
     * This function puts information above on SmartDashboard for quick viewing in Shuffleboard/Logging in AdvantageKit
     * 
     * <p> 
     * @param motor actual motor object
     * @param encoder actual encoder object
     */
    public void logMotorInfo(CANSparkFlex motor, RelativeEncoder encoder){  
        SmartDashboard.putNumber(motorName + "AppliedOutput", motor.getAppliedOutput());
        SmartDashboard.putNumber(motorName + "BusVoltage", motor.getBusVoltage());
        SmartDashboard.putNumber(motorName + "MotorTemp", motor.getMotorTemperature());
        SmartDashboard.putNumber(motorName + "OutputCurrent", motor.getOutputCurrent());
        SmartDashboard.putNumber(motorName + "Position", encoder.getPosition());
        SmartDashboard.putNumber(motorName + "Velocity", encoder.getVelocity());
    }
}
