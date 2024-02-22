package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSubsystem extends SubsystemBase {
    //public Limelight m_PhotonVision;
    public Limelight limelight;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    Rotation2d noRotation2d = new Rotation2d(0,0);
    Pose2d noPose2d = new Pose2d(0,0, noRotation2d);
    public double[] convertedCords = {0,0};
    public Pose2d AprilCords = new Pose2d(0,0, noRotation2d);
    public double currPercent = 1;

    


    public double[] gamePiecePos = {0,0};
    
    public SwerveSubsystem(Limelight llimelight) {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Canivore1);
        limelight = llimelight;
        gyro.getConfigurator();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
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

    public Pose2d getPose(){
        return swerveOdometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /* 
    public void resetOdometryAprilTag(){
        convertedCords = m_PhotonVision.getconvertCordinateSystems();
        Pose2d aprilCords = new Pose2d (convertedCords[0], convertedCords[1], noRotation2d);
        AprilCords = aprilCords;
        resetOdometry(aprilCords);
    }*/
    
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
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
    
    //Charge Station AutoBalancing
    //-------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------

    public void resetOdometryLLFieldCords() {
        if (limelight.limelightshooter.GetPipeline() == 2) {
            double[] rawcords = limelight.getFieldCordsTEST();
            Pose2d fieldcords = new Pose2d(rawcords[0], rawcords[1], getYaw());
            if (limelight.limelightshooter.HasTarget() != 0 || limelight.limelightright.HasTarget() != 0 || limelight.limelightleft.HasTarget() != 0) {
                resetOdometry(fieldcords);
            }
        }
        
    }


    @Override
    public void periodic(){

        swerveOdometry.update(getYaw(), getModulePositions());
        resetOdometryLLFieldCords();
        
        double[] OdometryArray = {getPose().getX(), getPose().getY(), getYaw().getDegrees()};
        SmartDashboard.putNumberArray("OdometryArray", OdometryArray);
        
        
        SmartDashboard.putNumber("Odometry X: ", OdometryArray[0]);
        SmartDashboard.putNumber("Odometry Y: ", OdometryArray[1]);
        //Game piece positions
        if (limelight.limelightshooter.GetPipeline() == 1) {
            limelight.limelightManger.GetClosestGamePiecePositions(OdometryArray, getYaw().getDegrees());
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
        angle = (angle < 0) ? 360 + angle: angle;

        double[] flywheelArray = {OdometryArray[0] + Constants.Pivot.pivotDistanceToRobotCenter * Math.cos(angle * (3.14159 / 180.0)), OdometryArray[1] + Constants.Pivot.pivotDistanceToRobotCenter * Math.sin(angle * (3.14159 / 180.0))};

        GlobalVariables.distanceToSpeaker = Math.sqrt((flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) * (flywheelArray[0] - Constants.Shooter.SUBWOOFERPositionX) + (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY) * (flywheelArray[1] - Constants.Shooter.SUBWOOFERPositionY));
        SmartDashboard.putNumber("Distance to speaker", GlobalVariables.distanceToSpeaker);

        SmartDashboard.putNumber("desired angle", (((66 + (-22) * Math.log(GlobalVariables.distanceToSpeaker) + 90.377)) / 360) *75 - 4.17);
        SmartDashboard.putNumber("desired velo", ((0.9621 * GlobalVariables.distanceToSpeaker + 24.4843) * (180/Math.PI)));
    }
}