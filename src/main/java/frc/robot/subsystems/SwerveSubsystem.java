package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
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
        
    //     AutoBuilder.configureHolonomic(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::setModuleSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                 new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // Translation PID constants
    //                 new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD), // Rotation PID constants
    //                 Constants.Swerve.maxSpeed, // Max module speed, in m/s
    //                 Constants.Swerve.driveBase, // Drive base radius in meters. Distance from robot center to furthest module.
    //                 new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
    //         ),
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
    poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), getPose());
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
                        
                                    for(SwerveModule mod : mSwerveMods){
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
    
    public void justTurn(){
        for (SwerveModule mod : mSwerveMods){
            mod.literallyJustTurnBro();
        }
    }

    public void changePercent(){
        if (currPercent == 1){
            currPercent = 0.5;
        } else if (currPercent == 0.5){
            currPercent = 1;
        }
    }

    public void changeHeadingState(){
        if (headingState == HeadingState.PICKUP){
            // headingState = HeadingState.AIMED;
            headingState = HeadingState.AIMED;
        } else if (headingState == HeadingState.AIMED){
            headingState = HeadingState.FREE;
        } else if (headingState == HeadingState.FREE){
            headingState = HeadingState.PICKUP;
        }
    }

    public void freeHeadingState(){
        headingState = HeadingState.FREE;
    }

    public Pose2d getPose(){
        return swerveOdometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public double getNominalYaw(){
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
    
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] states = getStates();
        ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(
            states[0], states[1], states[2], states[3]
        );
        return chassisSpeeds;
    }
    
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    
    public void zeroGyro(){
        gyro.setYaw(0);
    }
    
    public void setGyro(double degrees){
        gyro.setYaw(degrees);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
    
    public void resetOdometryLLFieldCords() {
        if (Limelight.limelightshooter.GetPipeline() == 2 && Math.abs(Limelight.limelightshooter.aprilTagResult[2]) < 3.5) {
            double[] rawcords = Limelight.limelightshooter.Fieldresult;
            Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYaw());
            AprilCords = fieldcords;
            if (Limelight.limelightshooter.HasTarget() != 0 || Limelight.limelightright.HasTarget() != 0 || Limelight.limelightleft.HasTarget() != 0) {
                resetOdometry(fieldcords);
                poseEstimator.addVisionMeasurement(fieldcords, Limelight.limelightshooter.limelightTable.getEntry("tl").getDouble(0));
            } 
            if (Limelight.limelightshooter.HasTarget() != 0 && Math.abs(Limelight.limelightshooter.aprilTagResult[2]) < 2.4) {
                resetOdometry(fieldcords);
                poseEstimator.resetPosition(getYaw(), getModulePositions(), fieldcords);
            }
        }
    }

    public double map(double val, double inMin, double inMax, double outMin, double outMax){
        return ((val-inMin)*(outMax-outMin)
                /(inMax-inMin))
                +outMin;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        resetOdometryLLFieldCords();
    
        double[] OdometryArray = {getPose().getX(), getPose().getY(), getYaw().getDegrees()};
        SmartDashboard.putNumberArray("OdometryArray", OdometryArray);
        
        // SmartDashboard.putNumber("Odometry X: ", OdometryArray[0]);
        // SmartDashboard.putNumber("Odometry Y: ", OdometryArray[1]);

        double[] poseEstimatorPos = {poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY(), poseEstimator.getEstimatedPosition().getRotation().getDegrees()};
        SmartDashboard.putNumberArray("pose estimator array", poseEstimatorPos);

        if (GlobalVariables.isEnabled == false) {
            poseEstimator.resetPosition(getYaw(), getModulePositions(), AprilCords);
            resetOdometry(AprilCords);
        }

        //Game piece positions
        if (Limelight.limelightshooter.GetPipeline() == 1) {
        Limelight.limelightManger.GetClosestGamePiecePositions(OdometryArray, getYaw().getDegrees());
        }

        

        double[] ypr = new double[3];
        ypr[0] = gyro.getYaw().getValueAsDouble();
        ypr[1] = gyro.getPitch().getValueAsDouble();
        ypr[2] = gyro.getRoll().getValueAsDouble();
        SmartDashboard.putNumber("yaw", ypr[0]);
        SmartDashboard.putNumber("gyro angle", ypr[1]);
        SmartDashboard.putNumber("roll", ypr[2]);
        SmartDashboard.putNumber("currPercent", currPercent);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Falcon degrees", mod.mAngleMotor.getPosition().getValueAsDouble());
        }

        double angle = gyro.getYaw().getValueAsDouble() % 360;
        angle = (angle < 0) ? 360 + angle : angle;

        // double[] flywheelArray = {OdometryArray[0] + Constants.Shooter.pivotDistanceToRobotCenter * Math.cos(angle * (3.14159 / 180.0)), OdometryArray[1] + Constants.Shooter.pivotDistanceToRobotCenter * Math.sin(angle * (3.14159 / 180.0))};

        double[] flywheelArray = {OdometryArray[0] + Constants.Pivot.pivotDistanceToRobotCenter * Math.cos(angle * (3.14159 / 180.0)), OdometryArray[1] + Constants.Pivot.pivotDistanceToRobotCenter * Math.sin(angle * (3.14159 / 180.0))};

        GlobalVariables.distanceToSpeaker = Math.sqrt((flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) * (flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) + (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY) * (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY));
        SmartDashboard.putNumber("Distance to speaker", GlobalVariables.distanceToSpeaker);

        SmartDashboard.putNumber("desired angle", ((((-22) * Math.log(GlobalVariables.distanceToSpeaker) + 90.377)) / 360) *75 - 4.17);
        double desiredvelo;
        if (GlobalVariables.distanceToSpeaker < 6) {
            desiredvelo = 30 * 91.7;
        } else {
            desiredvelo = (0.9621 * GlobalVariables.distanceToSpeaker + 24.4843) * (288/Math.PI);
        }
        SmartDashboard.putNumber("desired velo", desiredvelo);
    }
}