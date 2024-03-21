// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
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
		public static final double ampPosition = 10;
		public static final double stowPosition = 0;

		public static final int ampBarMotorID = 40;
		public static final boolean ampBarIsInverted = false;
		public static final IdleMode idleMode = IdleMode.kBrake;
		public static final int ampBarCurrLimit = 40; 
		
		public static final double ampBarkP = 0.15; 
		public static final double ampBarkI = 0.00125; 
		public static final double ampBarkIZone = 0.1;
		public static final double ampBarkD = 0.0;
		public static final double ampBarkFF = 0.0;
		public static final double ampBarMaxVelo = 4000; 
		public static final double ampBarMaxAcc = 4000;
		public static final double ampBarMaxError = 0.01; 

		public static final boolean shouldRestore = false;
		public static final boolean shouldBurn = false;
	}
	
	public static final class Climber {
		public static final int servoCenter = 1500; // in microseconds
		public static final int microsecondDegreesofFreedom = 300; // 40.5 angular degrees
		public static final int servoRightEngage = 1670;
		public static final int servoRightDisengage = 1350;
		public static final int servoLeftEngage = 1475;//servoCenter - (microsecondDegreesofFreedom / 2);
		public static final int servoLeftDisengage = 1700; //servoCenter + (microsecondDegreesofFreedom / 2);

		public static final int climberRightID = 35;
		public static final int climberLeftID = 36;
		public static final IdleMode idleMode = IdleMode.kBrake;

		public static final int servoRightID = 1;
		public static final int servoLeftID = 2;
		
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
		public static final double topAmpVelo = 3600;
		public static final double bottomAmpVelo = 2200;
		
		public static final int topFlywheelMotorID = 25;
		public static final boolean topIsInverted = true;
		public static final int bottomFlywheelMotorID = 26;
		public static final boolean bottomIsInverted = true;
		public static final IdleMode idleMode = IdleMode.kCoast;
				
		public static final double topFlywheelkP = 0.00000625;
		public static final double topFlywheelkI = 0.000000325*1.25;
		public static final double topFlywheelkIZone = 2000;
		public static final double topFlywheelkD = 0; 
		public static final double topFlywheelMaxVelo = 6500;
		public static final double topFlywheelMaxAcc = 6500;
		public static final double topFlywheelMaxError = 50; 
		
		public static final double bottomFlywheelkP = 0.0000003125; 
		public static final double bottomFlywheelkI = 0.000000325*1.25; 
		public static final double bottomFlywheelkIZone = 2000;
		public static final double bottomFlywheelkD = 0.0; 
		public static final double bottomFlywheelMaxVelo = 6500;
		public static final double bottomFlywheelMaxAcc = 6500;
		public static final double bottomFlywheelMaxError = 50;
	}
	
	public static final class Intake {
		public static final int intakeDriveMotorID = 20;
		public static final boolean intakeIsInverted = false;
		public static IdleMode idleMode = IdleMode.kBrake;
		public static final int intakeCurrentLimit = 50;
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
		public static final double stickDeadband = 0.1;
		
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
		public static final double subwooferPosition = 11;
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
		public static final double maxSpeed = 5.5;
		public static final double maxAngularVelocity = 4.25;

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
			public static final int driveMotorID = 1; //3
			public static final int angleMotorID = 2; //4
			public static final int canCoderID = 3; //9
			public static double angleOffset = (360-11.95)/360;//314.5 these aren't accurate just refrences
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 4; //7
			public static final int angleMotorID = 5; //8
			public static final int canCoderID = 6; //11
			public static double angleOffset = (27.86)/360;//246.7
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 7; //3
			public static final int angleMotorID = 8; //4
			public static final int canCoderID = 9; //9
			public static double angleOffset = (360-95.10)/360;//.47
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 10; //13
			public static final int angleMotorID = 11; //2
			public static final int canCoderID = 12; //12
			public static double angleOffset = (31.29)/360;//257.95
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}
	}
}