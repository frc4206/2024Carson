package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.RobotCentric driveRobotCentricNoDeadband = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    
    public enum HeadingState {
        FREE,
        BACKWARD,
        AIMED,
        AMPED,
        PICKUP
    }

    public HeadingState headingState = HeadingState.FREE;

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command applyRequestCommand(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void applyRequest(Supplier<SwerveRequest> requestSupplier) {
        this.setControl(requestSupplier.get());
    }

    private void fromChassisSpeeds(ChassisSpeeds speeds) {
        applyRequest(() -> driveRobotCentricNoDeadband.
            withVelocityX(speeds.vxMetersPerSecond).
            withVelocityY(speeds.vyMetersPerSecond).
            withRotationalRate(speeds.omegaRadiansPerSecond).
            withDriveRequestType(DriveRequestType.Velocity)
        );
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose,
            (pose) -> seedFieldRelative(pose),
            this::getCurrentRobotChassisSpeeds,
            (speeds) -> fromChassisSpeeds(speeds),
            new HolonomicPathFollowerConfig(
                new PIDConstants(10, 0.0, 0.0),
                new PIDConstants(10, 0.0, 0.0),
                Constants.Swerve.maxTranslationVelocity,
                driveBaseRadius,
                new ReplanningConfig(true, true)
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public void changeHeadingState(HeadingState newState){
        headingState = newState;
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
}
