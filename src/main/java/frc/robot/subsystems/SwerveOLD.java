package frc.robot.subsystems;

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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveOLD extends SubsystemBase {
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
    
    public enum HeadingState {
        PICKUP,
        BACKWARD,
        AIMED,
        AMPED,
        FREE
    }

    public HeadingState headingState = HeadingState.FREE;
    
    public SwerveOLD() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.canivoreName);
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
                new ReplanningConfig(true, false) // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE ClimberSide

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

    public void toggleAmped(){
        if (headingState != HeadingState.AMPED){
            headingState = HeadingState.AMPED;
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

        SmartDashboard.putNumberArray("odometry", actualOdo);
        SmartDashboard.putNumberArray("Pose estimator", OdometryArray);

        if (Limelight.limelightshooter.GetPipeline() == 1) {
            Limelight.limelightManger.GetClosestGamePiecePositions(OdometryArray, getYaw().getDegrees());
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        }

        GlobalVariables.Position.distanceToSpeaker = Math.sqrt(((Limelight.limelightshooter.aprilTagResult[0]) * (Limelight.limelightshooter.aprilTagResult[0]))    +      ((Limelight.limelightshooter.aprilTagResult[2]) * (Limelight.limelightshooter.aprilTagResult[2])));
        GlobalVariables.Pivot.desiredPosition = 15.25*Math.pow(.755, GlobalVariables.Position.distanceToSpeaker);

        SmartDashboard.putNumber("distance to speaker", GlobalVariables.Position.distanceToSpeaker);
        SmartDashboard.putNumberArray("limelight distance array", Limelight.limelightshooter.aprilTagResult);
        SmartDashboard.putNumber("DesPos", GlobalVariables.Pivot.desiredPosition);
    }
}