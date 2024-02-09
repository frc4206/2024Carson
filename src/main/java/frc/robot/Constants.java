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

  public static final class Swerve {
    public static final int pigeonID = 20; //30
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5);
    public static final double wheelBase = Units.inchesToMeters(23.75);
    public static final double wheelDiameter = Units.inchesToMeters(3.94);//change to 3.7ish for MK4s when sure 
    public static final double wheelCircumference = wheelDiameter * Math.PI;


    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); //6.86:1
    public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

    public static final double objDetectMaxPosError = 0.02;
    public static final double objDetectMaxRotationError = 1;

    public static final double disOdometryMaxPosError = 0.2; 
    public static final double disOdometryMasRotationError = 1;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* Angle Motor PID Values */
    public static final double angleKP = 1.2;
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

    public static final double objDetectyKP = 0.0005;
    public static final double objDetectyKI = 0.0;
    public static final double objDetectyKD = 0.0; 

    public static final double objDetectYawKP = 0.0003; 
    public static final double objDetectYawKI = 0.0;
    public static final double objDetectYawKD = 0.0; 

    /* Distance Odometry (2) PID Values */
    public static final double disOdometryxKP = 0.0005; 
    public static final double disOdometryxKI = 0.0;
    public static final double disOdometryxKD = 0.0;

    public static final double disOdometryYKP = 0.0005; 
    public static final double disOdometryYKI = 0.0; 
    public static final double disOdometryYKD = 0.0; 

    public static final double disOdometryYawKP = 0.0003; 
    public static final double disOdometryYawKI = 0.0; 
    public static final double disOdometryYawKD = 0.0; 

    /* PID To Game Piece PID Values */
    public static final double toGamePiecexKP = 0.0002; 
    public static final double toGamePiecexKI = 0.0; 
    public static final double toGamePiecexKD = 0.0; 

    public static final double toGamePieceyKP = 0.0; 
    public static final double toGamePieceyKI = 0.0; 
    public static final double toGamePieceyKD = 0.0; 

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
    public static final double maxSpeed = 4.5; //meters per second
    public static final double maxAngularVelocity = 11.5;

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
        public static final int driveMotorID = 7; //3
        public static final int angleMotorID = 8; //4
        public static final int canCoderID = 11; //9
        public static double angleOffset = 0.849;//314.5 these aren't accurate just refences
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 18; //7
        public static final int angleMotorID = 6; //8
        public static final int canCoderID = 10; //11
        public static double angleOffset = 0.2615;//246.7
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 3; //3
        public static final int angleMotorID = 4; //4
        public static final int canCoderID = 9; //9
        public static double angleOffset = 0.695;//.47
        public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 13; //13
        public static final int angleMotorID = 2; //2
        public static final int canCoderID = 12; //12
        public static double angleOffset = 0.492;//257.95
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
    public static final int IntakeDriveMotorID = 20;
    public static final int IntkeBeamBreakDIO = 0;
  }

  public static final class Shooter {
    public static final int ShooterLeadMotorID = 25;
    public static final int ShooterFollowerID = 26;
    public static final int ShooterPivotID = 27;

    public static final int ShooterBeamBreak = 1;

    public static final double SUBWOOFERPosition = 0;
    public static final double PODIUMPosition = 0;
    public static final double AMPLIFIERPosition = 0;
    public static final double WINGPosition = 0;
    
    /* Shooter Flywheel Values */
    public static final double flyWheelIZone = 0.0;

    public static final double flyWheelFF = 0.0; 

    public static final double flyWheelMaxVel = 1; 
    public static final int flyWheelMaxVelID = 0; 

    public static final double flyWheelMinVel = -1;
    public static final int flyWheelMinVelID = 0;

    public static final double flyWheelMaxAccel = 100; 
    public static final int flyWheelMaxAccelID = 0;

    public static final double flyWheelAllowedError = 5; 
    public static final int flyWheelAllowedErrorID = 0;

    public static final double flyWheelKP = 0.00029; 
    public static final double flyWheelKI = 7e-7; 
    public static final double flyWheelKD = 0.0; 

    /* Shooter Pivot Values */
    public static final double pivotFF = 0.0; 

    public static final double pivotMaxVel = 1; 
    public static final int pivotMaxVelID = 0; 

    public static final double pivotMinVel = -1; 
    public static final int pivotMinVelID = 0; 

    public static final double pivotMaxAccel = 100; 
    public static final int pivotMaxAccelID = 0; 

    public static final double pivotAllowedError = 5; 
    public static final int pivotAllowedErrorID = 0;

    public static final double pivotCurrLimit = 40; 

    public static final double pivotKP = 0.02; 
    public static final double pivotKI = 9e-8; 
    public static final double pivotKD = 0.0; 


  }

  public static final class Elevator {
    public static final int ElevatorLeaderID = 30;
    public static final int ElevatorFollowerID = 31;   
    public static final int ElevatorTopLimitSwitch = 2;
    public static final int ElevatorBottomLimitSwitch = 3;
    public static final int ElevatorGoToSetPoint = -100;

    /* Elevator motor PID Values + motor speeds*/
    public static final double elevKP = 0.02;
    public static final double elevKI = 9e-8;
    public static final double elevKD = 0.0;

    public static final double elevStopSpeed = 0.0;
    public static final double elevUpSpeed = 0.8;
    public static final double elevDownSpeed = -0.8;

    public static final double elevResetPosition = 7.5;
  }

  public static final class Climber {
    public static final int climberLeaderMotorID = 35;
    public static final int climberFollowerID = 36;
    public static final int climberLimitSwitch = 4;
    public static final int climberGoToSetPoint = -100;

    public static final double vortexClimberSubsystemLeadKP = 0.02; 
    public static final double vortexClimberSubsystemLeadKI = 9e-8; 
    public static final double vortexClimberSubsystemLeadKD = 0.0; 

    public static final double vortexClimberSubsystemLeadFF = 0.0; 
    //MaxVel, MinVel, and MaxAccel are under autoconstants 
    public static final int vortexClimberSubsystemMaxVelID = 0;
    public static final int ClimberLeadMotorID = 0;
    public static final int ClimberFollowerID = 0;

    public static final double climberResetPosition = 7.5;
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
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}