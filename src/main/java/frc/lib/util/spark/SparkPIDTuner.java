package frc.lib.util.spark;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <p>
 * This class is a shortcut to tune PIDs quickly without having to redeploy code through 
 * every change. Use this class in tandem with the SparkDefaultMethods interface to set 
 * different positions/velocities for the motor encoder to travel to.
 * 
 * <p> 
 * DO NOT USE THIS CLASS IF THE SUBSYSTEM CONTAINS A SPARKCONFIGURATION OBJECT.
 */
public class SparkPIDTuner {
    private String motorName;

    public static double motorkP = 0;
    public static double motorkI = 0;
    public static double motorkIZone = 0;
    public static double motorkD = 0;
    public static double motorkFF = 0;
    public static double motorkMaxVelo = 0;
    public static double motorkMaxAcc = 0;
    public static double motorkMaxError = 0;

    /**
     * <p>
     * Creates an instance of SparkPIDTuner. This object is a shortcut to tune PIDs quickly
     * without having to redeploy code through every change. Use this object in tandem with
     * the SparkDefaultMethods interface to set different positions/velocities for the motor
     * encoder to travel to.
     * 
     * <p> 
     * DO NOT USE THIS CLASS IF THE SUBSYSTEM CONTAINS A SPARKCONFIGURATION OBJECT.
     * 
     * @param motorName name of the motor (conveyorMotor => conveyor is the name to enter in the parameter)
     */
    public SparkPIDTuner(String motorName){
        this.motorName = motorName;
    }

    /**
     * <p>
     * Periodically updates the PID settings of the motor controller according to
     * the values inputted on SmartDashboard. In testing, this function should be called
     * in the periodic function of your subsystem.
     * 
     * <p>
     * The format for the SmartDashboard Constants is similar to 'motorName + kP'
     * <p>
     * Examples: conveyorkP, conveyorkI, conveyorkD
     * 
     * @param pidController actual PID controller object
     */
    public void updatePIDSettings(SparkPIDController pidController){
        motorkP = SmartDashboard.getNumber(motorName + "kP", 0);
        motorkI = SmartDashboard.getNumber(motorName + "kI", 0);
        motorkIZone = SmartDashboard.getNumber(motorName + "kIZone", 0);
        motorkD = SmartDashboard.getNumber(motorName + "kD", 0);
        motorkFF = SmartDashboard.getNumber(motorName + "kFF", 0);
        motorkMaxVelo = SmartDashboard.getNumber(motorName + "MaxVelo", 0);
        motorkMaxAcc = SmartDashboard.getNumber(motorName + "MaxAcc", 0);
        motorkMaxError = SmartDashboard.getNumber(motorName + "MaxError", 0);
        
        pidController.setP(motorkP);
        pidController.setI(motorkI);
        pidController.setIZone(motorkIZone);
        pidController.setD(motorkD);
        pidController.setFF(motorkFF);
        pidController.setOutputRange(-1, 1, 0);
        pidController.setSmartMotionMaxVelocity(motorkMaxVelo, 0);
        pidController.setSmartMotionMaxAccel(motorkMaxAcc, 0);
        pidController.setSmartMotionAllowedClosedLoopError(motorkMaxError, 0);
    }
}
