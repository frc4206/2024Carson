package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    
    /* Used by SwerveControllerCommand in Auto */
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

    public void changePercent(){
        if (currPercent == 1){
            currPercent = 0.5;
        } else if (currPercent == 0.5){
            currPercent = 1;
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

    public void resetOdometryLLFieldCords() {
        double distanceToApriTag = 0;
        if (Limelight.limelightshooter.GetPipeline() == 2) {
            double[] rawcords = Limelight.limelightshooter.fieldResult;
            Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYaw());
            AprilCords = fieldcords;
            
            if (Limelight.limelightshooter.HasTarget() != 0) {
                distanceToApriTag = Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1]));
                resetOdometry(fieldcords);
                poseEstimator.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));

            if (!aprilInit) {
                aprilInit = true;
                aprilStartTrackTime = Timer.getFPGATimestamp();
                previousPose = swerveOdometry.getPoseMeters();
            }
            aprilCurrTrackTime = Timer.getFPGATimestamp() - aprilStartTrackTime;

            if ( distanceToApriTag < 3 && aprilCurrTrackTime > .3) {
                //swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
                //poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
            }
            else if (distanceToApriTag < 4) {
                swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
                double yError = Math.abs(swerveOdometry.getPoseMeters().getY() - previousPose.getY());
                double xError = Math.abs(swerveOdometry.getPoseMeters().getX() - previousPose.getX());
                if ( ((yError < .3 && aprilCurrTrackTime > .3) || (Math.abs(swerveOdometry.getPoseMeters().getY() - poseEstimator.getEstimatedPosition().getY()) < .5 && yError < .1 && aprilCurrTrackTime > .1))       &&          (xError < .3 && aprilCurrTrackTime > .3) || (Math.abs(swerveOdometry.getPoseMeters().getX() - poseEstimator.getEstimatedPosition().getX()) < .5 && xError < .1 && aprilCurrTrackTime > .1)) {
                    poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
                }
            }
            SmartDashboard.putNumber("distance to apriltag", distanceToApriTag);
            previousPose = AprilCords;
            } else {
                aprilInit = false;
                aprilCurrTrackTime = 0;
                aprilStartTrackTime = 0;
                badpose = true;
                
        }
        } 
        SmartDashboard.putNumber("AprilTag time since tracking", aprilCurrTrackTime);
    }

    public void resetOdometryRED() {
        double distanceToApriTag = 0;
        if (Limelight.limelightshooter.GetPipeline() == 2) {
            double[] rawcords = Limelight.limelightshooter.fieldResult;
            Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYaw());
            AprilCords = fieldcords;
            
            if (Limelight.limelightshooter.HasTarget() != 0) {
                distanceToApriTag = Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1]));
                resetOdometry(fieldcords);
                poseInvertEstimator.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));

            if (!aprilInit) {
                aprilInit = true;
                aprilStartTrackTime = Timer.getFPGATimestamp();
                previousPose = swerveInvertOdometry.getPoseMeters();
            }
            aprilCurrTrackTime = Timer.getFPGATimestamp() - aprilStartTrackTime;

            if ( distanceToApriTag < 3 && aprilCurrTrackTime > .3) {
                //swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
                //poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
            }
            else if (distanceToApriTag < 4) {
                swerveInvertOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
                double yError = Math.abs(swerveInvertOdometry.getPoseMeters().getY() - previousPose.getY());
                double xError = Math.abs(swerveInvertOdometry.getPoseMeters().getX() - previousPose.getX());
                if ( ((yError < .3 && aprilCurrTrackTime > .3) || (Math.abs(swerveInvertOdometry.getPoseMeters().getY() - poseInvertEstimator.getEstimatedPosition().getY()) < .5 && yError < .1 && aprilCurrTrackTime > .1))       &&          (xError < .3 && aprilCurrTrackTime > .3) || (Math.abs(swerveInvertOdometry.getPoseMeters().getX() - poseInvertEstimator.getEstimatedPosition().getX()) < .5 && xError < .1 && aprilCurrTrackTime > .1)) {
                    poseInvertEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
                }
            }
            SmartDashboard.putNumber("distance to apriltag", distanceToApriTag);
            previousPose = AprilCords;
            } else {
                aprilInit = false;
                aprilCurrTrackTime = 0;
                aprilStartTrackTime = 0;
                badpose = true;
                
        }
        } 
        SmartDashboard.putNumber("AprilTag time since tracking", aprilCurrTrackTime);
    }

    public void resetOdometryLLFieldCordsInverted(){
        boolean reset = false;
        if (Limelight.limelightshooter.GetPipeline() == 2 && Math.abs(Limelight.limelightshooter.aprilTagResult[2]) < 3.5) {
            double[] rawcords = Limelight.limelightshooter.fieldResult;
            Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYawInverted());
            AprilCords = fieldcords;
            if (Limelight.limelightshooter.HasTarget() != 0) {
                resetOdometryInverted(fieldcords);
                reset = true;
                poseInvertEstimator.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));
                //poseInvertEstimator.resetPosition(getYawInverted(), getModulePositions(), fieldcords);
                SmartDashboard.putBoolean("is resetting", reset);
                if (Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1])) < 3) {
                    swerveInvertOdometry.resetPosition(getYaw(), getModulePositionsInverted(), AprilCords);
                    poseInvertEstimator.resetPosition(getYawInverted(), getModulePositions(), AprilCords);
                }
                SmartDashboard.putNumber("distance to apriltag", Math.abs(Math.sqrt(Limelight.limelightshooter.aprilTagResult[2] * Limelight.limelightshooter.aprilTagResult[2]) + (Limelight.limelightshooter.aprilTagResult[1] * Limelight.limelightshooter.aprilTagResult[1])));
            }
        }
    }

    public double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return ((val-inMin)*(outMax-outMin)
            /(inMax-inMin))
            +outMin;
    }

    @Override
    public void periodic() {
        double[] actualOdo = {0,0,0};

        if (GlobalVariables.alliance == Alliance.Blue){
            swerveOdometry.update(getYaw(), getModulePositions());
            poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
            OdometryArray[2] = getYaw().getDegrees();
            resetOdometryLLFieldCords();
            OdometryArray[0] = poseEstimator.getEstimatedPosition().getX();
            OdometryArray[1] = poseEstimator.getEstimatedPosition().getY();
            double[] actualOdoBlue = {swerveOdometry.getPoseMeters().getX(), swerveOdometry.getPoseMeters().getY(), getYaw().getDegrees()};
            actualOdo = actualOdoBlue;
            if (Limelight.limelightshooter.HasTarget() != 0 && GlobalVariables.isEnabled == false){
                poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
                swerveOdometry.resetPosition(getYaw(), getModulePositions(), AprilCords);
            }
        } else if (GlobalVariables.alliance == Alliance.Red){
            swerveInvertOdometry.update(getYawInverted(), getModulePositions());
            poseInvertEstimator.updateWithTime(Timer.getFPGATimestamp(), getYawInverted(), getModulePositionsInverted());
            OdometryArray[2] = getYawInverted().getDegrees();
            resetOdometryRED();
            double[] actualOdoRed = {swerveInvertOdometry.getPoseMeters().getX(), swerveInvertOdometry.getPoseMeters().getY(), getYawInverted().getDegrees()};
            actualOdo = actualOdoRed;
            OdometryArray[0] = poseInvertEstimator.getEstimatedPosition().getX();
            OdometryArray[1] = poseInvertEstimator.getEstimatedPosition().getY();
            if (Limelight.limelightshooter.HasTarget() != 0 && GlobalVariables.isEnabled == false){
                swerveInvertOdometry.resetPosition(getYaw(), getModulePositionsInverted(), AprilCords);
                poseInvertEstimator.resetPosition(getYawInverted(), getModulePositions(), AprilCords);
            }
        }
        

        SmartDashboard.putNumberArray("odometry", actualOdo);
        SmartDashboard.putNumberArray("Pose estimator", OdometryArray);

        //Game piece positions
        if (Limelight.limelightshooter.GetPipeline() == 1) {
            Limelight.limelightManger.GetClosestGamePiecePositions(OdometryArray, getYaw().getDegrees());
        }

        

        double[] ypr = new double[3];
        ypr[0] = gyro.getYaw().getValueAsDouble();
        ypr[1] = gyro.getPitch().getValueAsDouble();
        ypr[2] = gyro.getRoll().getValueAsDouble();
        // SmartDashboard.putNumber("yaw", ypr[0]);
        // SmartDashboard.putNumber("gyro angle", ypr[1]);
        // SmartDashboard.putNumber("roll", ypr[2]);
        // SmartDashboard.putNumber("currPercent", currPercent);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Falcon degrees", mod.mAngleMotor.getPosition().getValueAsDouble());
        }

        double angle = gyro.getYaw().getValueAsDouble() % 360;
        angle = (angle < 0) ? 360 + angle : angle;

        // double[] flywheelArray = {OdometryArray[0] + Constants.Shooter.pivotDistanceToRobotCenter * Math.cos(angle * (3.14159 / 180.0)), OdometryArray[1] + Constants.Shooter.pivotDistanceToRobotCenter * Math.sin(angle * (3.14159 / 180.0))};

        double[] flywheelArray = {OdometryArray[0] + Constants.Pivot.pivotDistanceToRobotCenter * Math.cos(angle * (3.14159 / 180.0)), OdometryArray[1] + Constants.Pivot.pivotDistanceToRobotCenter * Math.sin(angle * (3.14159 / 180.0))};

        GlobalVariables.distanceToSpeaker = Math.sqrt((flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) * (flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) + (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY) * (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY));
        // SmartDashboard.putNumber("Distance to speaker", GlobalVariables.distanceToSpeaker);

        GlobalVariables.desiredAngle = ((((-22) * Math.log(GlobalVariables.distanceToSpeaker) + 90.377)) / 360) *75 - 4.17;

        if (GlobalVariables.distanceToSpeaker < 6) {
            desiredVelo = 30 * 91.7;
        } else {
            desiredVelo = (0.9621 * GlobalVariables.distanceToSpeaker + 24.4843) * (288/Math.PI);
        }
        GlobalVariables.desiredVelo = desiredVelo;
    }
}