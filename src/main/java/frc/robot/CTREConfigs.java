package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        //Angle
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyTimeThreshold = Constants.Swerve.anglePeakCurrentDuration;
        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        angleSupplyLimit.SupplyCurrentThreshold = Constants.Swerve.anglePeakCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.Slot0.kS = Constants.Swerve.angleKF;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

        //Drive
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyTimeThreshold = Constants.Swerve.drivePeakCurrentDuration;
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentThreshold = Constants.Swerve.drivePeakCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kS = Constants.Swerve.driveKF;        

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        //Cancoder
        // swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
}