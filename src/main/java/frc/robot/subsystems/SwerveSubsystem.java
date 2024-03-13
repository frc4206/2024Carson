package frc.robot.subsystems;

import org.ejml.equation.MatrixConstructor;
import org.opencv.objdetect.FaceDetectorYN;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDriveOdometry swerveInvertOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    Rotation2d noRotation2d = new Rotation2d(0,0);
    Pose2d noPose2d = new Pose2d(0,0, noRotation2d);
    public double[] convertedCords = {0,0};
    public Pose2d AprilCords = new Pose2d(0,0, noRotation2d);
    public double currPercent = 1;
    public double[] gamePiecePos = {0,0};
    double nomYaw = 0;
    double realYaw = 0;
    double rotations = 0;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDrivePoseEstimator poseInvertEstimator;
    double[] OdometryArray = new double[3];
    double maxJitter = 0.4;
    

    boolean aprilInit = false;
    double aprilStartTrackTime = 0;
    double aprilCurrTrackTime = 0;
    boolean badpose = true;
    Pose2d previousPose;
    

    double desiredVelo;

    public enum HeadingState {
        PICKUP,
        BACKWARD,
        AIMED,
        FREE
    }

    public HeadingState headingState = HeadingState.FREE;
    
    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Canivore1);
        gyro.getConfigurator();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), getPose());
        swerveInvertOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYawInverted(), getModulePositionsInverted());
        poseInvertEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYawInverted(), getModulePositionsInverted(), getPoseInverted());

        /* 
         private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);
*/

        //Matrix<N3, N1> test = new Matrix<>(VecBuilder.fill(0.01, 0.01, 0.01));
        //poseEstimator.setVisionMeasurementStdDevs(test);
        
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setModuleSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(Constants.Swerve.driveKP, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(Constants.Swerve.angleKP, 0.0, 0.0), // Rotation PID constants
                    Constants.Swerve.maxSpeed, // Max module speed, in m/s
                    Constants.Swerve.wheelBase, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation, 
                getYaw()
            )
            : new ChassisSpeeds(
                translation.getX(), 
                translation.getY(), 
                rotation)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
            
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    } 
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setModuleSpeeds(ChassisSpeeds desiredSpeeds){
        SwerveDriveKinematics.desaturateWheelSpeeds(Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredSpeeds), Constants.Swerve.maxSpeed);

        int i = 0;
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredSpeeds)[i], false);
            i += 1;
        }
    }

    public void changeHeadingState() {
        if (headingState == HeadingState.PICKUP){
            // headingState = HeadingState.AIMED;
            headingState = HeadingState.AIMED;
        } else if (headingState == HeadingState.AIMED){
            headingState = HeadingState.FREE;
        } else if (headingState == HeadingState.FREE){
            headingState = HeadingState.PICKUP;
        }
    }

    public void toggleSource(){
        if (headingState != HeadingState.PICKUP){
            headingState = HeadingState.PICKUP;
        } else {
            headingState = HeadingState.FREE;
        }
    }

    public void toggleAimed(){
        if (headingState != HeadingState.AIMED){
            headingState = HeadingState.AIMED;
        } else {
            headingState = HeadingState.FREE;
        }
    }

    public void freeHeadingState() {
        headingState = HeadingState.FREE;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    
    public Pose2d getPoseInverted(){
        return swerveInvertOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetOdometryInverted(Pose2d pose){
        swerveInvertOdometry.resetPosition(getYaw(), getModulePositionsInverted(), pose);
    }

    public double getNominalYaw() {
        realYaw = getYaw().getDegrees();
        rotations = Math.round((realYaw/360));
        if (realYaw < 0){
            nomYaw = -realYaw;
            if (nomYaw > 360){
                nomYaw += (360*rotations);
            }
            if (nomYaw < 0){
                nomYaw += 360;
            }
            nomYaw = 360 - nomYaw;
        } else {
            nomYaw = realYaw - (360*rotations);
            if (nomYaw < 0){
                nomYaw += 360;
            }
        }
        return nomYaw;
    }
    
    public double getNominalYawInverted(){
        realYaw = getYawInverted().getDegrees();
        rotations = Math.round((realYaw/360));
        if (realYaw < 0){
            nomYaw = -realYaw;
            if (nomYaw > 360){
                nomYaw += (360*rotations);
            }
            if (nomYaw < 0){
                nomYaw += 360;
            }
            nomYaw = 360 - nomYaw;
        } else {
            nomYaw = realYaw - (360*rotations);
            if (nomYaw < 0){
                nomYaw += 360;
            }
        }
        return nomYaw;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = getStates();
        ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(
            states[0], states[1], states[2], states[3]
        );
        return chassisSpeeds;
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    
    public SwerveModulePosition[] getModulePositionsInverted(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPositionInverted();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }
    
    public void setGyro(double degrees) {
        gyro.setYaw(degrees);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
    
    public Rotation2d getYawInverted(){
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble() - 180);
    }


    // public void resetWithApriltags(SwerveDriveOdometry swerveOdo, SwerveDrivePoseEstimator poseEst, SwerveModulePosition[] modPos, Rotation2d yaw) {
    //     double distanceToApriTag = 0;
    //     if (Limelight.limelightshooter.GetPipeline() == 2) {
    //         double[] rawcords = Limelight.limelightshooter.fieldResult;
    //         Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYaw());
    //         AprilCords = fieldcords;
            
    //         if (Limelight.limelightshooter.HasTarget() != 0) {
    //             distanceToApriTag = Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1]));
    //             resetOdometry(fieldcords);
    //             poseEst.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));

            
            
    //         if (distanceToApriTag < 4) {
    //             if (!aprilInit) {
    //                 aprilInit = true;
    //                 aprilStartTrackTime = Timer.getFPGATimestamp();
    //                 previousPose = swerveOdo.getPoseMeters();
    //             }

    //             aprilCurrTrackTime = Timer.getFPGATimestamp() - aprilStartTrackTime;
    //             swerveOdo.resetPosition(yaw, modPos, AprilCords);
    //             double yError = Math.abs(swerveOdo.getPoseMeters().getY()) - Math.abs(previousPose.getY());
    //             double xError = Math.abs(swerveOdo.getPoseMeters().getX()) - Math.abs(previousPose.getX());
    //             SmartDashboard.putNumber("xError", xError);
    //             SmartDashboard.putNumber("yError", yError);

    //             if (yError < maxJitter && xError < maxJitter) {
    //                 if (aprilCurrTrackTime > .3) {
    //                     poseEst.resetPosition(yaw, modPos, AprilCords);
    //                     System.out.println(" RESETI SPAGETTI");
    //                 }
    //                 //if (yError + xError > .2 || aprilCurrTrackTime > 5) {
    //                 //    aprilCurrTrackTime = 0;
    //                 //    aprilInit = false;
    //                 //}
    //             } else {
    //                 aprilCurrTrackTime = 0;
    //                 aprilInit = false;
    //             }
    //         }
    //         SmartDashboard.putNumber("distance to apriltag", distanceToApriTag);
    //         previousPose = swerveOdo.getPoseMeters();
    //         } else {
    //             aprilInit = false;
    //             aprilCurrTrackTime = 0;
    //             aprilStartTrackTime = 0;
    //             badpose = true;
                
    //     }
    //     } 
    //     SmartDashboard.putNumber("AprilTag time since tracking", aprilCurrTrackTime);
    // }

    // public void resetOdometryLLFieldCords() {
    //     double distanceToApriTag = 0;
    //     if (Limelight.limelightshooter.GetPipeline() == 2) {
    //         double[] rawcords = Limelight.limelightshooter.fieldResult;
    //         Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYaw());
    //         AprilCords = fieldcords;
            
    //         if (Limelight.limelightshooter.HasTarget() != 0) {
    //             distanceToApriTag = Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1]));
    //             resetOdometry(fieldcords);
    //             poseEstimator.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));

    //         if (!aprilInit) {
    //             aprilInit = true;
    //             aprilStartTrackTime = Timer.getFPGATimestamp();
    //             previousPose = swerveOdometry.getPoseMeters();
    //         }
    //         aprilCurrTrackTime = Timer.getFPGATimestamp() - aprilStartTrackTime;

    //         if ( distanceToApriTag < 3 && aprilCurrTrackTime > .3) {
    //             //swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
    //             //poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
    //         }
    //         else if (distanceToApriTag < 4) {
    //             swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
    //             double yError = Math.abs(swerveOdometry.getPoseMeters().getY() - previousPose.getY());
    //             double xError = Math.abs(swerveOdometry.getPoseMeters().getX() - previousPose.getX());
    //             if ( ((yError < maxJitter && aprilCurrTrackTime > .3) || (Math.abs(swerveOdometry.getPoseMeters().getY() - poseEstimator.getEstimatedPosition().getY()) < .5 && yError < maxJitter && aprilCurrTrackTime > .1))       &&          (xError < .3 && aprilCurrTrackTime > .3) || (Math.abs(swerveOdometry.getPoseMeters().getX() - poseEstimator.getEstimatedPosition().getX()) < .5 && xError < maxJitter && aprilCurrTrackTime > .1)) {
    //                 poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
    //                 if (yError + xError > .2 || aprilCurrTrackTime > 5) {
    //                     aprilCurrTrackTime = 0;
    //                     aprilInit = false;
    //                 }
    //             }
    //         }
    //         SmartDashboard.putNumber("distance to apriltag", distanceToApriTag);
    //         previousPose = AprilCords;
    //         } else {
    //             aprilInit = false;
    //             aprilCurrTrackTime = 0;
    //             aprilStartTrackTime = 0;
    //             badpose = true;
                
    //     }
    //     } 
    //     SmartDashboard.putNumber("AprilTag time since tracking", aprilCurrTrackTime);
    // }

    // public void resetOdometryRED() {
    //     double distanceToApriTag = 0;
    //     if (Limelight.limelightshooter.GetPipeline() == 2) {
    //         double[] rawcords = Limelight.limelightshooter.fieldResult;
    //         Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYaw());
    //         AprilCords = fieldcords;
            
    //         if (Limelight.limelightshooter.HasTarget() != 0) {
    //             distanceToApriTag = Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1]));
    //             resetOdometry(fieldcords);
    //             poseInvertEstimator.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));

    //         if (!aprilInit) {
    //             aprilInit = true;
    //             aprilStartTrackTime = Timer.getFPGATimestamp();
    //             previousPose = swerveInvertOdometry.getPoseMeters();
    //         }
    //         aprilCurrTrackTime = Timer.getFPGATimestamp() - aprilStartTrackTime;

    //         if ( distanceToApriTag < 3 && aprilCurrTrackTime > .3) {
    //             //swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
    //             //poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
    //         }
    //         else if (distanceToApriTag < 4) {
    //             swerveInvertOdometry.resetPosition(getYawInverted(), getModulePositionsInverted(), AprilCords);
    //             double yError = Math.abs(swerveInvertOdometry.getPoseMeters().getY() - previousPose.getY());
    //             double xError = Math.abs(swerveInvertOdometry.getPoseMeters().getX() - previousPose.getX());
    //             if ( yError < maxJitter  &&  xError < maxJitter  && aprilCurrTrackTime > 0.3) {
    //                 poseInvertEstimator.resetPosition(getYawInverted(), getModulePositionsInverted(), AprilCords);
    //                 if (yError + xError > .2 || Math.abs(swerveInvertOdometry.getPoseMeters().getY() - poseInvertEstimator.getEstimatedPosition().getY()) < .5 ) {
    //                     aprilCurrTrackTime = 0;
    //                     aprilInit = false;
    //                 }
    //             }
    //         }
    //         SmartDashboard.putNumber("distance to apriltag", distanceToApriTag);
    //         previousPose = AprilCords;
    //         } else {
    //             aprilInit = false;
    //             aprilCurrTrackTime = 0;
    //             aprilStartTrackTime = 0;
    //             badpose = true;
                
    //     }
    //     } 
    //     SmartDashboard.putNumber("AprilTag time since tracking", aprilCurrTrackTime);
    // }

    // public void resetOdometryLLFieldCordsInverted(){
    //     boolean reset = false;
    //     if (Limelight.limelightshooter.GetPipeline() == 2 && Math.abs(Limelight.limelightshooter.aprilTagResult[2]) < 3.5) {
    //         double[] rawcords = Limelight.limelightshooter.fieldResult;
    //         Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYawInverted());
    //         AprilCords = fieldcords;
    //         if (Limelight.limelightshooter.HasTarget() != 0) {
    //             resetOdometryInverted(fieldcords);
    //             reset = true;
    //             poseInvertEstimator.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));
    //             //poseInvertEstimator.resetPosition(getYawInverted(), getModulePositions(), fieldcords);
    //             SmartDashboard.putBoolean("is resetting", reset);
    //             if (Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1])) < 3) {
    //                 swerveInvertOdometry.resetPosition(getYaw(), getModulePositionsInverted(), AprilCords);
    //                 poseInvertEstimator.resetPosition(getYawInverted(), getModulePositions(), AprilCords);
    //             }
    //             SmartDashboard.putNumber("distance to apriltag", Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1])));
    //         }
    //     }
    // }

    public double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return ((val-inMin)*(outMax-outMin)
            /(inMax-inMin))
            +outMin;
    }

    public boolean shooterShouldRun(){
        if (GlobalVariables.alliance == Alliance.Blue){
            return (getPose().getX()) < 5;
        } else {
            return (getPose().getX()) > Constants.Field.fieldLength - 5;
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());


        double[] rawcords = Limelight.limelightshooter.fieldResult;
        AprilCords = new Pose2d(rawcords[0], rawcords[1], getYaw());
        
        

        OdometryArray[2] = getYaw().getDegrees();
        OdometryArray[0] = poseEstimator.getEstimatedPosition().getX();
        OdometryArray[1] = poseEstimator.getEstimatedPosition().getY();
        

        double[] actualOdo = {swerveOdometry.getPoseMeters().getX(), swerveOdometry.getPoseMeters().getY(), getYaw().getDegrees()};
        // if (GlobalVariables.alliance == Alliance.Blue){
        //    swerveOdometry.update(getYaw(), getModulePositions());
        //    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        //    OdometryArray[2] = getYaw().getDegrees();
        //    resetOdometryLLFieldCords();
        //    OdometryArray[0] = poseEstimator.getEstimatedPosition().getX();
        //    OdometryArray[1] = poseEstimator.getEstimatedPosition().getY();
        //    double[] actualOdoBlue = {swerveOdometry.getPoseMeters().getX(), swerveOdometry.getPoseMeters().getY(), getYaw().getDegrees()};
        //    actualOdo = actualOdoBlue;
        //    if (Limelight.limelightshooter.HasTarget() != 0 && GlobalVariables.isEnabled == false){
        //        poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
        //        swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
        //    }
        // } else if (GlobalVariables.alliance == Alliance.Red){
        //    swerveInvertOdometry.update(getYawInverted(), getModulePositionsInverted());
        //    poseInvertEstimator.updateWithTime(Timer.getFPGATimestamp(), getYawInverted(), getModulePositionsInverted());
        //    OdometryArray[2] = getYawInverted().getDegrees();
        //    resetWithApriltags(swerveInvertOdometry, poseInvertEstimator, getModulePositionsInverted(), getYawInverted());
        //    double[] actualOdoRed = {swerveInvertOdometry.getPoseMeters().getX(), swerveInvertOdometry.getPoseMeters().getY(), getYawInverted().getDegrees()};
        //    actualOdo = actualOdoRed;
        //    OdometryArray[0] = poseInvertEstimator.getEstimatedPosition().getX();
        //    OdometryArray[1] = poseInvertEstimator.getEstimatedPosition().getY();
        //    if (Limelight.limelightshooter.HasTarget() != 0 && GlobalVariables.isEnabled == false){
        //        swerveInvertOdometry.resetPosition(getYaw(), getModulePositionsInverted(), AprilCords);
        //        poseInvertEstimator.resetPosition(getYawInverted(), getModulePositionsInverted(), AprilCords);
        //    }
        // }
        

        SmartDashboard.putNumberArray("odometry", actualOdo);
        SmartDashboard.putNumberArray("Pose estimator", OdometryArray);

        if (Limelight.limelightshooter.GetPipeline() == 1) {
            Limelight.limelightManger.GetClosestGamePiecePositions(OdometryArray, getYaw().getDegrees());
        }

        double[] ypr = new double[3];
        ypr[0] = gyro.getYaw().getValueAsDouble();
        ypr[1] = gyro.getPitch().getValueAsDouble();
        ypr[2] = gyro.getRoll().getValueAsDouble();

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        }

        double angle = gyro.getYaw().getValueAsDouble() % 360;
        angle = (angle < 0) ? 360 + angle : angle;

        //double[] flywheelArray = {OdometryArray[0] + Constants.Pivot.pivotDistanceToRobotCenter * Math.cos(angle * (3.14159 / 180.0)), OdometryArray[1] + Constants.Pivot.pivotDistanceToRobotCenter * Math.sin(angle * (3.14159 / 180.0))};
        //GlobalVariables.distanceToSpeaker = Math.sqrt((flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) * (flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) + (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY) * (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY));
        //GlobalVariables.desiredAngle = ((((-22) * Math.log(GlobalVariables.distanceToSpeaker) + 90.377)) / 360) *75 - 4.17;

        //if (GlobalVariables.distanceToSpeaker < 6) {
        //    desiredVelo = 30 * 91.7;
        //} else {
        //    desiredVelo = (0.9621 * GlobalVariables.distanceToSpeaker + 24.4843) * (288/Math.PI);
        //}
        //GlobalVariables.desiredVelo = desiredVelo;
        double distanceToSpeaker = Math.sqrt(((Limelight.limelightshooter.aprilTagResult[0]) * (Limelight.limelightshooter.aprilTagResult[0]))    +      ((Limelight.limelightshooter.aprilTagResult[2]) * (Limelight.limelightshooter.aprilTagResult[2])));
        double DesiredPivot = 14.3599*Math.pow(.755, distanceToSpeaker);
        SmartDashboard.putNumber("distance to speaker", distanceToSpeaker);
        SmartDashboard.putNumberArray("limelight distance array", Limelight.limelightshooter.aprilTagResult);

        SmartDashboard.putNumber("DesPos", DesiredPivot);
    }
}