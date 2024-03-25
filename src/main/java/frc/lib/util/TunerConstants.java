package frc.lib.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSubsystem;

// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);

    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(50).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final double kSlipCurrentA = 80.0;
    private static final double kSupplyCurrentA = 40;

    public static final double kSpeedAt12VoltsMps = 6.21;

    private static final double kCoupleRatio = 3;

    private static final double kDriveGearRatio = 5.142857142857142;
    private static final double kSteerGearRatio = 12.8;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = false;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "Canivore1";
    private static final int kPigeonId = 55;

    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 3;
    private static final double kFrontLeftEncoderOffset = 0.04052734375;
    private static final double kFrontLeftXPosInches = 9.5;
    private static final double kFrontLeftYPosInches = 10.5;

    private static final int kFrontRightDriveMotorId = 4;
    private static final int kFrontRightSteerMotorId = 5;
    private static final int kFrontRightEncoderId = 6;
    private static final double kFrontRightEncoderOffset = 0.40771484375;
    private static final double kFrontRightXPosInches = 9.5;
    private static final double kFrontRightYPosInches = -10.5;

    private static final int kBackLeftDriveMotorId = 7;
    private static final int kBackLeftSteerMotorId = 8;
    private static final int kBackLeftEncoderId = 9;
    private static final double kBackLeftEncoderOffset = 0.261962890625;
    private static final double kBackLeftXPosInches = -9.5;
    private static final double kBackLeftYPosInches = 10.5;

    private static final int kBackRightDriveMotorId = 10;
    private static final int kBackRightSteerMotorId = 11;
    private static final int kBackRightEncoderId = 12;
    private static final double kBackRightEncoderOffset = 0.415283203125;
    private static final double kBackRightXPosInches = -9.5;
    private static final double kBackRightYPosInches = -10.5;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final SwerveSubsystem DriveTrain = new SwerveSubsystem(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
