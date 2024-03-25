// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.spark.sparkConfig.FeedbackConfig;
import frc.lib.util.spark.sparkConfig.MotorConfig;
import frc.lib.util.spark.sparkConfig.PIDConfig;
import frc.lib.util.spark.sparkConfig.SparkConfig;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final String canivoreName = "Canivore1";
	
	public static final class AmpBar {
		public static final double stowPosition = 0;
		public static final double ampPosition = 1/3;

		public static final int ampLimitSwitchID = 1;
		public static final int hallZeroLimitSwitchID = 2;
		public static final int zeroLimitSwitchID = 3;


		public static final int ampBarMotorID = 40;
		public static final boolean ampBarIsInverted = false;
		public static final IdleMode idleMode = IdleMode.kBrake;
		public static final int ampBarCurrLimit = 40; 
		
		public static final double ampBarkP = 0.25;//0.45; 
		public static final double ampBarkI = 0.00375; 
		public static final double ampBarkIZone = 0.1;
		public static final double ampBarkD = 0.0;
		public static final double ampBarkFF = 0.0;
		public static final double ampBarMaxVelo = 5000; 
		public static final double ampBarMaxAcc = 5000;
		public static final double ampBarMaxError = 0.01; 

		public static final boolean shouldRestore = true;
		public static final boolean shouldBurn = true;

		
		public static final SparkConfig ampBarConfig = new SparkConfig(
			new FeedbackConfig(Feedback.defaultMinDuty, Feedback.defaultMaxDuty, ampBarMaxVelo, ampBarMaxAcc, ampBarMaxError), 
			new MotorConfig(ampBarMotorID, ampBarIsInverted, idleMode, ampBarCurrLimit), 
			new PIDConfig(ampBarkP, ampBarkI, ampBarkIZone, ampBarkD, ampBarkFF), 
			shouldRestore,
			shouldBurn
		);
	}
	
	public static final class Climber {
		public static final int servoCenter = 1500; // in microseconds
		public static final int microsecondDegreesofFreedom = 300; // 40.5 angular degrees
		public static final int servoEngage = 1670;
		public static final int servoDisengage = 1350;

		public static final int climberMotorID = 35;
		public static final boolean climberIsInverted = false;
		public static final IdleMode idleMode = IdleMode.kBrake;
		public static final int climberCurrentLimit = 100;

		public static final int servoID = 1;
		
		public static final double climberkP = 0.002;
		public static final double climberkI = 0.00002;
		public static final double climberkIZone = 0.1;
		public static final double climberkD = 0;
		public static final double climberkFF = 0;
		public static final double climberMaxVelo = 4000;
		public static final double climberMaxAcc = 4000;
		public static final double climberMaxError = 1;

		public static final boolean shouldRestore = false;
		public static final boolean shouldBurn = true;


		public static final SparkConfig climberConfig = new SparkConfig(
			new FeedbackConfig(Feedback.defaultMinDuty, Feedback.defaultMaxDuty, climberMaxVelo, climberMaxAcc, climberMaxError), 
			new MotorConfig(climberMotorID, climberIsInverted, idleMode, climberCurrentLimit), 
			new PIDConfig(climberkP, climberkI, climberkIZone, climberkD, climberkFF), 
			shouldRestore, 
			shouldBurn
		);
	}
	
	public static final class Conveyor {
		public static final int conveyorMotorID = 29;
		public static final boolean conveyorIsInverted = true;
		public static final IdleMode idleMode = IdleMode.kBrake;
		public static final int conveyorCurrentLimit = 40;

		public static final int conveyerBeamBreakID = 0;
		
		public static final double conveyorkP = 0.05;
		public static final double conveyorkI = 0.00005;
		public static final double conveyorkIzone = 5;
		public static final double conveyorkD = 0;
		public static final double conveyorkFF = 0;
		public static final double conveyorMaxVelo = 5000;
		public static final double conveyorMaxAcc = 5000;
		public static final double conveyorMaxError = 0.05;

		public static final boolean shouldRestore = false;
		public static final boolean shouldBurn = false;


		public static final SparkConfig conveyorConfig = new SparkConfig(
			new FeedbackConfig(Constants.Feedback.defaultMinDuty, Constants.Feedback.defaultMaxDuty, conveyorMaxVelo, conveyorMaxAcc, conveyorMaxError),
			new MotorConfig(conveyorMotorID, conveyorIsInverted, idleMode, conveyorCurrentLimit),
			new PIDConfig(conveyorkP, conveyorkI, conveyorkIzone, conveyorkD, conveyorkFF),
			shouldRestore,
			shouldBurn
		);
	}
	
	public static final class Feedback {
		public static final double defaultMinDuty = -1;
		public static final double defaultMaxDuty = 1;
	}

	public static final class Field {
		public static final double fieldLength = 16.452;
		public static final double fieldWidth = 8.211;
		public static final double subwooferLength = Units.inchesToMeters(36.125);
	}

	public static final class Flywheel {
		public static final double speakerVelo = 6500;
		public static final double topAmpVelo = 3850;
		public static final double bottomAmpVelo = 2350;
		
		public static final int topFlywheelMotorID = 25;
		public static final boolean topIsInverted = true;
		public static final int bottomFlywheelMotorID = 26;
		public static final boolean bottomIsInverted = true;
		public static final IdleMode idleMode = IdleMode.kCoast;
				
		public static final double topFlywheelkP = 0.00000625;
		public static final double topFlywheelkI = 0.000000325*1.25;
		public static final double topFlywheelkIZone = 0;
		public static final double topFlywheelkD = 0; 
		public static final double topFlywheelkFF = 0;
		public static final double topFlywheelMaxVelo = 6500;
		public static final double topFlywheelMaxAcc = 6500;
		public static final double topFlywheelMaxError = 50; 
		
		public static final double bottomFlywheelkP = 0.0000003125; 
		public static final double bottomFlywheelkI = 0.000000325*1.25; 
		public static final double bottomFlywheelkIZone = 0;
		public static final double bottomFlywheelkD = 0; 
		public static final double bottomFlywheelkFF = 0;
		public static final double bottomFlywheelMaxVelo = 6500;
		public static final double bottomFlywheelMaxAcc = 6500;
		public static final double bottomFlywheelMaxError = 50;

		public static final boolean shouldRestore = false;
		public static final boolean shouldBurn = true;


		public static final SparkConfig topFlywheelConfig = new SparkConfig(
			new FeedbackConfig(Feedback.defaultMinDuty, Feedback.defaultMaxDuty, topFlywheelMaxVelo, topFlywheelMaxAcc, topFlywheelMaxError), 
			new MotorConfig(topFlywheelMotorID, topIsInverted, idleMode), 
			new PIDConfig(topFlywheelkP, topFlywheelkI, topFlywheelkIZone, topFlywheelkD, topFlywheelkFF), 
			shouldRestore, 
			shouldBurn
		);

		public static final SparkConfig bottomFlywheelConfig = new SparkConfig(
			new FeedbackConfig(Feedback.defaultMinDuty, Feedback.defaultMaxDuty, bottomFlywheelMaxVelo, bottomFlywheelMaxAcc, bottomFlywheelMaxError), 
			new MotorConfig(bottomFlywheelMotorID, bottomIsInverted, idleMode), 
			new PIDConfig(bottomFlywheelkP, bottomFlywheelkI, bottomFlywheelkIZone, bottomFlywheelkD, bottomFlywheelkFF), 
			shouldRestore, 
			shouldBurn
		); 
	}
	
	public static final class Intake {
		public static final int intakeDriveMotorID = 20;
		public static final boolean intakeIsInverted = false;
		public static final IdleMode idleMode = IdleMode.kBrake;
		public static final int intakeCurrentLimit = 60;

		public static final boolean shouldRestore = false;
		public static final boolean shouldBurn = false;

		public static final SparkConfig intakeConfig = new SparkConfig(
			new MotorConfig(intakeDriveMotorID, intakeIsInverted, idleMode, intakeCurrentLimit), 
			shouldRestore,
			shouldBurn
		);
	}
	
	public static final class LEDS{
		public static final int LEDPort = 9;
		public static final int numLEDs = 48;

		public static final int highZero = 400;
		public static final int lowZero = 850;
		public static final int highOne = 800;
		public static final int lowOne = 450;
	}
	
	public static final class Limelight {
		public static final int limelightFrontCamID = 1; 
		public static final double limelightFrontAngle = 25; 
		public static final double limelightFrontHeight = 24.5; 
		public static final double limelightFrontTargetHeight = 0; 
		
		public static final int limelightLeftCamID = 2;
		public static final double limelightLeftAngle = 25; 
		public static final double limelightLeftHeight = 24.5; 
		public static final double limelightLeftTargetHeight = 0; 
		
		public static final int limelightRightCamID = 3; 
		public static final double limelightRightAngle = 25; 
		public static final double limelightRightHeight = 24.5; 
		public static final double limelightRightTargetHeight = 0; 
	}

	public static final class OperatorConstants {
		public static final double joystickDeadzone = 0.1;
		public static final double triggerDeadzone = 0.05;
		
		public static final int drivaPort = 0;
		public static final int operataPort = 1;
		public static final int operata2Port = 2;
		public static final int shootertestaPort = 3;
		public static final int ampBartestaPort = 4;
	}

	public static final class Pivot {
		public static final int pivotMotorID = 27;
		public static final boolean pivotIsInverted = false;
		public static final IdleMode idleMode = IdleMode.kBrake;
		public static final int pivotCurrentLimit = 40;
		public static final double pivotClosedLoopRampRate = 0.25;
		
		public static final double allowableThreshold = 0.5;

		public static final double ampPosition = 10;
		public static final double closePosition = 6;
		public static final double podiumPosition = 3.900;
		public static final double underPosition = 2.64;
		public static final double stagePosition = 1.73;
		public static final double subwooferPosition = 10.75;
		public static final double passPosition = 8;
		public static final double defaultPosition = 2;
		
		public static final double pivotkP = 0.15; 
		public static final double pivotkI = 0.00125; 
		public static final double pivotkIZone = 0.1;
		public static final double pivotkD = 0.0;
		public static final double pivotkFF = 0.0;
		public static final double pivotMaxVel = 4000; 
		public static final double pivotMinVel = -4000; 
		public static final double pivotMaxAccel = 4000;
		public static final double pivotAllowedError = 0.001; 
		public static final double pivotCurrLimit = 35; 

		public static final boolean shouldRestore = true;
		public static final boolean shouldBurn = false;


		public static final SparkConfig pivotConfig = new SparkConfig(
			new FeedbackConfig(Constants.Feedback.defaultMinDuty, Constants.Feedback.defaultMaxDuty, pivotMaxVel, pivotMaxAccel, pivotAllowedError), 
			new MotorConfig(pivotMotorID, pivotIsInverted, idleMode, pivotCurrentLimit, pivotClosedLoopRampRate), 
			new PIDConfig(pivotkP, pivotkI, pivotkIZone, pivotkD, pivotkFF), 
			shouldRestore, 
			shouldBurn
		);
	}
	
	public static final class Swerve {
		public static final int pigeonID = 55;
		public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(20.6);
		public static final double wheelBase = Units.inchesToMeters(18.74);
		public static final double wheelDiameter = Units.inchesToMeters(4); 
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double translationMultiplier = 1.25;
		public static final double rotationMultiplier = 0.75;

		public static final double openLoopRamp = 0.25;
		public static final double closedLoopRamp = 0.0;

		public static final double driveGearRatio = (5.14 / 1.0); //6.86:1
		public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

		public static final double objDetectMaxPosError = 0.02;
		public static final double objDetectMaxRotationError = 1;

		public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
			new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

		/* Swerve Current Limiting */
		public static final int angleStatorCurrentLimit = 40;
		public static final int angleSupplyCurrentLimit = 35;
		public static final boolean angleEnableStatorLimit = true;
		public static final boolean angleEnableSupplyLimit = true; 

		public static final int driveStatorCurrentLimit = 55;
		public static final int driveSupplyCurrentLimit = 50;
		public static final boolean driveEnableStatorLimit = true;
		public static final boolean driveEnableSupplyLimit = true;

		/* Angle Motor PID Values */
		public static final double angleKP = 1.1;
		public static final double angleKI = 0.0;
		public static final double angleKD = 0.0;
		public static final double angleKF = 0.0;

		/* Drive Motor PID Values */
		public static final double driveKP = 0.15;
		public static final double driveKI = 0.0;
		public static final double driveKD = 0.0;
		public static final double driveKF = 0.0;

		/* Object Detection PID Values */
		public static final double objDetectxKP = 0.0005;
		public static final double objDetectxKI = 0.0;
		public static final double objDetectxKD = 0.0;

		public static final double objDetectYKP = 0.0005;
		public static final double objDetectYKI = 0.0;
		public static final double objDetectYKD = 0.0; 

		public static final double objDetectYawKP = 0.0003; 
		public static final double objDetectYawKI = 0.0;
		public static final double objDetectYawKD = 0.0; 

		/* Distance Odometry (2) PID Values */
		public static final double disOdometryxKP = 0.3; 
		public static final double disOdometryxKI = 0.00001;
		public static final double disOdometryxKD = 0.0;

		public static final double disOdometryYKP = 0.3; 
		public static final double disOdometryYKI = 0.00001; 
		public static final double disOdometryYKD = 0.0;     
		
		public static final double disOdometryYawKP = 0.05; 
		public static final double disOdometryYawKI = 0.0; 
		public static final double disOdometryYawKD = 0.0; 

		public static final double disOdometryMaxPosError = 0.05; 
		public static final double disOdometryMaxRotationError = 1;

		/* PID To Game Piece PID Values */
		public static final double toGamePiecexKP = 0.0002; 
		public static final double toGamePiecexKI = 0.0; 
		public static final double toGamePiecexKD = 0.0; 

		public static final double toGamePieceYKP = 0.0; 
		public static final double toGamePieceYKI = 0.0; 
		public static final double toGamePieceYKD = 0.0; 

		public static final double toGamePieceYawKP = 0.05; 
		public static final double toGamePieceYawKI = 0.0; 
		public static final double toGamePieceYawKD = 0.0; 

		/* Drive Motor Characterization Values */
		//public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
		//public static final double driveKV = (2.44 / 12);
		//public static final double driveKA = (0.27 / 12);
		public static final double driveKS = (0); //divide by 12 to convert from volts to percent output for CTRE
		public static final double driveKV = (0);
		public static final double driveKA = (0);

		/* Swerve Profiling Values */
		public static final double maxSpeed = 6.22; //6.00 when FOC, 6.22 when non-FOC
		public static final double maxAngularVelocity = 4;

		/* Neutral Modes */
		public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
		public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

		/* Motor Inverts */
		public static final boolean driveMotorInvert = false;
		public static final boolean angleMotorInvert = false;

		/* Angle Encoder Invert */
		public static final int canCoderInvert = 0; //0-CCW, 1-ClockWise

		/* Module Specific Constants */
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final int driveMotorID = 1; 
			public static final int angleMotorID = 2; 
			public static final int canCoderID = 3;
			public static double angleOffset = (360-11.95)/360;
			public static final frc.lib.util.SwerveModuleConstants constants =
				new frc.lib.util.SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 4;
			public static final int angleMotorID = 5;
			public static final int canCoderID = 6;
			public static double angleOffset = (27.86)/360;
			public static final frc.lib.util.SwerveModuleConstants constants =
				new frc.lib.util.SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 7; 
			public static final int angleMotorID = 8; 
			public static final int canCoderID = 9;
			public static double angleOffset = (360-95.10)/360;
			public static final frc.lib.util.SwerveModuleConstants constants =
				new frc.lib.util.SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 10; 
			public static final int angleMotorID = 11; 
			public static final int canCoderID = 12;
			public static double angleOffset = (31.29)/360;
			public static final frc.lib.util.SwerveModuleConstants constants =
				new frc.lib.util.SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}





		
		private static final double steerkP = 100;
		private static final double steerkI = 0;
		private static final double steerkD = 0.2;
		private static final double steerkS = 0;
		private static final double steerkV = 1.5;
		private static final double steerkA = 0;

		private static final double drivekP = 50;
		private static final double drivekI = 0;
		private static final double drivekD = 0;
		private static final double drivekS = 0;
		private static final double drivekV = 0;
		private static final double drivekA = 0;

		private static final Slot0Configs steerGains = new Slot0Configs()
        	.withKP(steerkP).withKI(steerkI).withKD(steerkD)
        	.withKS(steerkS).withKV(steerkV).withKA(steerkA
		);

		private static final Slot0Configs driveGains = new Slot0Configs()
			.withKP(drivekP).withKI(drivekI).withKD(drivekD)
			.withKS(drivekS).withKV(drivekV).withKA(drivekA
		);

		private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
		private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

		private static final double slipCurrent = 80;
		private static final double supplyCurrent = 40;

		public static final double maxTranslationVelocity = 6.21;
		public static final double maxRotationalVelocity = 1.5 * Math.PI;

		private static final double coupleRatio = 3;

		private static final double driveRatio = 5.142857142857142;
		private static final double steerRatio = 12.8;
		private static final double wheelRadius = 2;

		private static final boolean steerInverted = false;
		private static final boolean leftIsInverted = false;
		private static final boolean rightIsInverted = true;

		private static final String CANBusName = "Canivore1";
		private static final int PigeonID = 55;

		private static final double steerInertia = 0.00001;
		private static final double driveInertia = 0.001;
		private static final double steerFrictionVoltage = 0.25;
		private static final double driveFrictionVoltage = 0.25;

		public static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
			.withPigeon2Id(PigeonID)
			.withCANbusName(CANBusName
		);

		private static final SwerveModuleConstantsFactory constantsCreator = new SwerveModuleConstantsFactory()
			.withDriveMotorGearRatio(driveRatio)
			.withSteerMotorGearRatio(steerRatio)
			.withWheelRadius(wheelRadius)
			.withSlipCurrent(slipCurrent)
			.withSteerMotorGains(steerGains)
			.withDriveMotorGains(driveGains)
			.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
			.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
			.withSpeedAt12VoltsMps(maxTranslationVelocity)
			.withSteerInertia(steerInertia)
			.withDriveInertia(driveInertia)
			.withSteerFrictionVoltage(steerFrictionVoltage)
			.withDriveFrictionVoltage(driveFrictionVoltage)
			.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
			.withCouplingGearRatio(coupleRatio)
			.withSteerMotorInverted(steerInverted
		);


		private static final int FLDriveMotorID = 1;
		private static final int FLSteerMotorID = 2;
		private static final int FLEncoderID = 3;
		private static final double FLEncoderOffset = 0.04052734375;
		private static final double FLXPosition = 9.5;
		private static final double FLYPosition = 10.5;

		private static final int FRDriveMotorID = 4;
		private static final int FRSteerMotorID = 5;
		private static final int FREncoderID = 6;
		private static final double FREncoderOffset = 0.40771484375;
		private static final double FRXPosition = 9.5;
		private static final double FRYPosition = -10.5;

		private static final int BLDriveMotorID = 7;
		private static final int BLSteerMotorID = 8;
		private static final int BLEncoderID = 9;
		private static final double BLEncoderOffset = 0.261962890625;
		private static final double BLXPosition = -9.5;
		private static final double BLYPosition = 10.5;

		private static final int BRDriveMotorID = 10;
		private static final int BRSteerMotorID = 11;
		private static final int BREncoderID = 12;
		private static final double BREncoderOffset = 0.415283203125;
		private static final double BRXPosition = -9.5;
		private static final double BRYPosition = -10.5;


		public static final SwerveModuleConstants FrontLeft = constantsCreator.createModuleConstants(
				FLSteerMotorID, FLDriveMotorID, FLEncoderID, FLEncoderOffset, Units.inchesToMeters(FLXPosition), Units.inchesToMeters(FLYPosition), leftIsInverted);
		public static final SwerveModuleConstants FrontRight = constantsCreator.createModuleConstants(
				FRSteerMotorID, FRDriveMotorID, FREncoderID, FREncoderOffset, Units.inchesToMeters(FRXPosition), Units.inchesToMeters(FRYPosition), rightIsInverted);
		public static final SwerveModuleConstants BackLeft = constantsCreator.createModuleConstants(
				BLSteerMotorID, BLDriveMotorID, BLEncoderID, BLEncoderOffset, Units.inchesToMeters(BLXPosition), Units.inchesToMeters(BLYPosition), leftIsInverted);
		public static final SwerveModuleConstants BackRight = constantsCreator.createModuleConstants(
				BRSteerMotorID, BRDriveMotorID, BREncoderID, BREncoderOffset, Units.inchesToMeters(BRXPosition), Units.inchesToMeters(BRYPosition), rightIsInverted);
	}
}