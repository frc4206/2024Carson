// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
		public static final double stickDeadband = 0.1;
		public static final String Canivore1 = "Canivore1";

	public static final class Field {
		public static final double fieldLength = 16.452;
		public static final double fieldWidth = 8.211;
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
		public static final int angleStatorCurrentLimit = 35;
		public static final int angleSupplyCurrentLimit = 40;
		public static final boolean angleEnableStatorLimit = true;
		public static final boolean angleEnableSupplyLimit = true; 

		public static final int driveStatorCurrentLimit = 55;
		public static final int driveSupplyCurrentLimit = 65;
		public static final boolean driveEnableStatorLimit = true;
		public static final boolean driveEnableSupplyLimit = true;

		/* Angle Motor PID Values */
		public static final double angleKP = 1.1;
		public static final double angleKI = 0.0;
		public static final double angleKD = 0.0;
		public static final double angleKF = 0.0;

		/* Drive Motor PID Values */
		public static final double driveKP = 0.10;
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

		public static final double toGamePieceYawKP = 0.0002; 
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
		public static final double maxSpeed = 5.25; //meters per second
		public static final double maxAngularVelocity = 4.875;

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
			public static double angleOffset = (360-11.68)/360;//314.5 these aren't accurate just refrences
			public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    	}


		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 4; //7
			public static final int angleMotorID = 5; //8
			public static final int canCoderID = 6; //11
			public static double angleOffset = (30.23)/360;//246.7
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 7; //3
			public static final int angleMotorID = 8; //4
			public static final int canCoderID = 9; //9
			public static double angleOffset = (360-95.09)/360;//.47
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}


		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 10; //13
			public static final int angleMotorID = 11; //2
			public static final int canCoderID = 12; //12
			public static double angleOffset = (157.67)/360;//257.95
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

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

	public static final class Intake {
		public static final int intakeDriveMotorID = 20;
		public static final int intakeFollowerMotorID = 21;

		public static final boolean driverInverted = true;
		public static final boolean followerInverted = true;
	}

	public static final class Shooter {
		public static final int shooterLeadMotorID = 25;
		public static final int shooterFollowerID = 26;
		public static final int shooterPivotID = 27;
	
		public static final double SUBWOOFERPositionX = 0;
		public static final double SUBWOOFERPositionY = 5.51;
		
		public static final double topFlyWheelKP = 0.0005; 
		public static final double topFlyWheelKI = 0.00001; 
		public static final double topFlyWheelKIZone = 100;
		public static final double topFlyWheelKD = 0.0; 
		public static final double topFlyWheelMaxVel = 6500;
		public static final double topFlyWheelMinVel = -6500;
		public static final double topFlyWheelMaxAccel = 100;
		public static final double topFlyWheelAllowedError = 5; 

		public static final double bottomFlyWheelKP = 0.0005; 
		public static final double bottomFlyWheelKI = 0.00001; 
		public static final double bottomFlyWheelKIZone = 100;
		public static final double bottomFlyWheelKD = 0.0; 
		public static final double bottomFlyWheelMaxVel = 6500;
		public static final double bottomFlyWheelMinVel = -6500;
		public static final double bottomFlyWheelMaxAccel = 100;
		public static final double bottomFlyWheelAllowedError = 5;

		public static final double MIN_OUTPUT_RANGE = -1;
		public static final double MAX_OUTPUT_RANGE = 1;
		public static final int SLOT_ID_GAIN = 0; /* Between 0 and 3 */
  	}

	public static final class Pivot {
		public static final int pivotMotorID = 27;

		public static final double ampPosition = 8;
		public static final double closePosition = 4.828;
		public static final double spikePosition = 4.450;
		public static final double podiumPosition = 3.900;
		public static final double underPosition = 2.6;

		public static final double stagePosition = 1.73;
		public static final double wingPosition = 1.0;

		/* Shooter Pivot Values */
		public static final double pivotKP = 0.15; 
		public static final double pivotKI = 0.00125; 
		public static final double pivotKIZone = 0.1;
		public static final double pivotKD = 0.0;
		public static final double pivotMaxVel = 4000; 
		public static final double pivotMinVel = -4000; 
		public static final double pivotMaxAccel = 4000;
		public static final double pivotAllowedError = 0.001; 
		public static final double pivotCurrLimit = 35; 

		public static final double pivotDistanceToRobotCenter = 3.25;
	}

	public static final class Elevator {
		public static final int elevatorLeaderID = 30;
		public static final int elevatorFollowerID = 31;
		public static final boolean elevatorLeaderisInverted = true;
		public static final boolean elevatorFollowisInverted = false;
		public static final int elevatorTopLimitSwitch = 2;
		public static final int elevatorBottomLimitSwitch = 3;
    
		public static final double elevKP = 0.00025;
		public static final double elevKI = 0.000005;
		public static final double elevKIZone = 30;
		public static final double elevKD = 0.0;

		public static final double elevStopSpeed = 0.0;
		public static final double elevUpSpeed = 0.55;
		public static final double elevDownSpeed = -0.55;

		public static final double elevResetPosition = 7.5;
		public static final double elevHighPosition = 100;
		public static final double elevTrapPosition = 90;
		public static final double elevAmpPosition = 80;
	}	

	public static final class Climber {
		public static final int climberRightLeadID = 36;
		public static final int climberLeftFollowID = 35;
		public static final int servoRightID = 1;
		public static final int servoLeftID = 0;
		public static final int climberLimitSwitchID = 4;

		public static final double climberkP = 0.002;
		public static final double climberkI = 0.00002;
		public static final double climberkIZone = 0.1;
		public static final double climberkD = 0;
		public static final double climberAllowedError = 1;
		public static final double climberMaxVelo = 4000;
		public static final double climberMaxAcc = 4000;

		/** Since the servos on our climber are facing opposite
		 * directions, the engage and disengage values are inverted **/
		public static final double servoPosRightEngage = 0.2d;
		public static final double servoPosRightDisEngage = 0.0d;
		public static final double servoPosLeftEngage = 0.0d;
		public static final double servoPosLeftDisEngage = 0.2d;

		public static final double climberTopSetpoint = 15;
		public static final double climberBottomSetpoint = 0;
	}

	public static final class LEDS{
		public static final int candleID = 60;
		public static final int numLEDs = 37;
		public static final int ledStartOffset = 0;
	}
  
	public static final class Conveyor {
		public static final int conveyorMotorID = 29;
		public static final int conveyerBeamBreakID = 0;
		public static final boolean conveyorInverted = true;

		public static final double conveyorkP = 0.0225;
		public static final double conveyorkI = 0.0005;
		public static final double conveyorkIzone = 0;
		public static final double conveyorkD = 0;
		public static final double conveyorMaxVelo = 6000;
		public static final double conveyorMaxAcc = 6000;
		public static final double conveyorMaxError = 0.1;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxSpeedMetersPerSecondfast = 5;
		public static final double kMaxAccelerationMetersPerSecondSquaredfast = 5;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPXControllerfast = 9;
		public static final double kPYControllerfast = 9;
		public static final double kPThetaController = 3.6;

		// Constraint for the motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
			new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
			);
	}
}