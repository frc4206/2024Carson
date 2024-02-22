package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.HeadingState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveSubsystem s_Swerve;
    private XboxController controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    private double botYaw;
    private double yawSet;
    private double errorYaw;
    private double outputYaw;
    private PIDController pidyaw = new PIDController(0.01, 0, 0);
    private PIDController pidyawi = new PIDController(0.005, 0, 0);
    private double rAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(SwerveSubsystem s_Swerve, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        for (SwerveModule mod : s_Swerve.mSwerveMods){
            mod.mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    public void execute() {
        botYaw = s_Swerve.getNominalYaw();

        switch (s_Swerve.headingState){
            case PICKUP:
                if (GlobalVariables.alliance == Alliance.Blue){
                    if (botYaw > 0 && botYaw < 180){
                        yawSet = -30; //makes robot overshoot and go to else statement
                    } else {
                        yawSet = 300;
                    }
                } else if (GlobalVariables.alliance == Alliance.Red){
                    if (botYaw > 0 && botYaw < 180){
                        yawSet = 60;
                    } else {
                        yawSet = -390; //makes robot overshoot and go to else statement
                    }
                }
                break;
            case AIMED:
                yawSet= Math.toDegrees(
                            Math.atan(
                                (s_Swerve.getPose().getY()-5.52)/
                                (s_Swerve.getPose().getX()-0)
                            )
                        );
                break;
            case BACKWARD:
                if (botYaw > 0 && botYaw < 180){
                        yawSet = 0; //makes robot overshoot and go to else statement
                    } else {
                        yawSet = 360;
                    }
                break;
            case FREE:
                yawSet = 0;
                break;
            default: 
                yawSet = 0;
                break;
        }

        // if (yawSet < 0){
        //     errorYaw = botYaw + yawSet;
        // } else {
        //     errorYaw = botYaw - yawSet;
        // }
        if (s_Swerve.headingState == HeadingState.AIMED){
            if (yawSet < 0){
                errorYaw = botYaw - (360 + yawSet);
                if (Math.abs(errorYaw) > 1.5) { 
                    outputYaw = pidyaw.calculate(botYaw, 360 + yawSet);
                } else {
                    outputYaw = pidyawi.calculate(botYaw, 360 + yawSet);
                }
            } else {
                errorYaw = botYaw - yawSet;
                if (Math.abs(errorYaw) > 1.5) {
                    outputYaw = pidyaw.calculate(botYaw, yawSet);
                } else {
                    outputYaw = pidyawi.calculate(botYaw, yawSet);
                }
            }
        } else {
            if (Math.abs(errorYaw) > 1.5) { 
                outputYaw = pidyaw.calculate(botYaw, 360 + yawSet);
            } else {
                outputYaw = pidyawi.calculate(botYaw, 360 + yawSet);
            }
        }

        double yAxisDeadzoned;
        double xAxisDeadzoned;
        double yAxis = (-controller.getRawAxis(translationAxis)*Constants.Swerve.translationMultiplier);
        double xAxis = (-controller.getRawAxis(strafeAxis)*Constants.Swerve.translationMultiplier);

        switch (s_Swerve.headingState){
            case PICKUP:
                rAxis = outputYaw;
                break;
            case AIMED:
                rAxis = outputYaw;
                break;
            case BACKWARD:
                rAxis = outputYaw;
            case FREE:
                rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
                break;
            default:
                rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
                break;
        }
        
        // double[] newCoords = s_Swerve.apply_deadzone(xAxis, yAxis, Constants.stickDeadband);
        // xAxisDeadzoned = newCoords[0];
        // yAxisDeadzoned = newCoords[1];

        yAxisDeadzoned = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : s_Swerve.map(Math.abs(yAxis), Constants.stickDeadband, 1.0, 0.0, 1.0);
        yAxisDeadzoned = yAxis >= 0.0 ? yAxisDeadzoned : -yAxisDeadzoned;
        xAxisDeadzoned = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : s_Swerve.map(Math.abs(xAxis), Constants.stickDeadband, 1.0, 0.0, 1.0);
        xAxisDeadzoned = xAxis >= 0.0 ? xAxisDeadzoned : -xAxisDeadzoned;
        
        yAxisDeadzoned = yAxisDeadzoned * yAxisDeadzoned; //(Math.cos(Math.PI*(yAxisDeadzoned + 1.0d)/2.0d)) + 0.5d;
        yAxisDeadzoned = yAxis >= 0.0 ? yAxisDeadzoned : -yAxisDeadzoned; 
        xAxisDeadzoned = xAxisDeadzoned * xAxisDeadzoned; //(Math.cos(Math.PI*(xAxisDeadzoned + 1.0d)/2.0d)) + 0.5d;
        xAxisDeadzoned = xAxis >= 0.0 ? xAxisDeadzoned : -xAxisDeadzoned;


        if (s_Swerve.headingState == HeadingState.FREE){
            rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        } else if (s_Swerve.headingState == HeadingState.AIMED){
            // rAxis = -rAxis;
        }

        translation = new Translation2d(yAxisDeadzoned, xAxisDeadzoned).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

        SmartDashboard.putNumber("yawNominal", botYaw);
        SmartDashboard.putNumber("yawSet", yawSet);
        SmartDashboard.putNumber("yawOutput", outputYaw);
        SmartDashboard.putNumber("yawError", errorYaw);
    }
}