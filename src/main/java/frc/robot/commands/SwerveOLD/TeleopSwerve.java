package frc.robot.commands.SwerveOLD;

import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveOLD;
import frc.robot.subsystems.SwerveOLD.HeadingState;
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
    
    private SwerveOLD s_Swerve;
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
    private PIDController pidyaw2 = new PIDController(0.100, 0, 0);
    private double rAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(SwerveOLD s_Swerve, XboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        for(SwerveModule mod : s_Swerve.mSwerveMods) {
            mod.mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    public void execute() {
        botYaw = s_Swerve.getNominalYaw();

        switch (s_Swerve.headingState){
            case PICKUP:
                if (GlobalVariables.alliance == Alliance.Blue) {
                    if (botYaw > 0 && botYaw < 180) {
                        yawSet = -30; //makes robot overshoot and go to else statement
                    } else {
                        yawSet = 300;
                    }
                } else if (GlobalVariables.alliance == Alliance.Red) {
                    if (botYaw > 0 && botYaw < 180) {
                        yawSet = 60;
                    } else {
                        yawSet = -390; //makes robot overshoot and go to else statement
                    }
                }
                if (Math.abs(errorYaw) > 1.5) { 
                    outputYaw = pidyaw.calculate(botYaw, yawSet);
                } else {
                    outputYaw = pidyawi.calculate(botYaw, yawSet);
                }
                rAxis = outputYaw;
                break;
            case AIMED:
                // yawSet= Math.toDegrees(
                //     Math.atan(
                //         (s_Swerve.getPose().getY()-5.72)/
                //         (s_Swerve.getPose().getX()-0)
                //     )
                // );
                // if (GlobalVariables.alliance == Alliance.Blue){
                //     yawSet = Math.atan2(s_Swerve.getPose().getY() - 5.72, s_Swerve.getPose().getX());
                //     if (yawSet < 0) {
                //         errorYaw = botYaw - (360 + yawSet);
                //         if(Math.abs(errorYaw) > 1.5) { 
                //             outputYaw = pidyaw.calculate(botYaw, 360 + yawSet);
                //         } else {
                //             outputYaw = pidyawi.calculate(botYaw, 360 + yawSet);
                //         }
                //     } else {
                //         errorYaw = botYaw - yawSet;
                //         if(Math.abs(errorYaw) > 1.5) {
                //             outputYaw = pidyaw.calculate(botYaw, yawSet);
                //         } else {
                //             outputYaw = pidyawi.calculate(botYaw, yawSet);
                //         }
                //     }
                // } else if (GlobalVariables.alliance == Alliance.Red){
                //     yawSet = Math.atan2(s_Swerve.getPose().getY() - 5.72, Constants.Field.fieldLength-s_Swerve.getPose().getX());
                //     if (yawSet < 0) {
                //         errorYaw = botYawInverted - (360 + yawSet);
                //         if(Math.abs(errorYaw) > 1.5) { 
                //             outputYaw = pidyaw.calculate(botYawInverted, 360 + yawSet);
                //         } else {
                //             outputYaw = pidyawi.calculate(botYawInverted, 360 + yawSet);
                //         }
                //     } else {
                //         errorYaw = botYawInverted - yawSet;
                //         if(Math.abs(errorYaw) > 1.5) {
                //             outputYaw = pidyaw.calculate(botYawInverted, yawSet);
                //         } else {
                //             outputYaw = pidyawi.calculate(botYawInverted, yawSet);
                //         }
                //     }
                // }
                // rAxis = outputYaw;
                break;
            case AMPED:
                if (GlobalVariables.alliance == Alliance.Blue) {
                    if (botYaw > 0 && botYaw < 270) {
                        yawSet = 90;
                    } else {
                        yawSet = -90; //makes robot overshoot and go to positive setpoint
                    }
                } else if (GlobalVariables.alliance == Alliance.Red) {
                    if (botYaw > 90 && botYaw < 360) {
                        yawSet = 270;
                    } else {
                        yawSet = -90; //makes robot overshoot and go to positive setpoint
                    }
                }
                if (Math.abs(errorYaw) > 1.5) { 
                    outputYaw = pidyaw.calculate(botYaw, yawSet);
                } else {
                    outputYaw = pidyawi.calculate(botYaw, yawSet);
                }
                rAxis = outputYaw;
                break;
            case BACKWARD:
                if(botYaw > 0 && botYaw < 180) {
                        yawSet = 0;
                    } else {
                        yawSet = 360;
                    }
                if (Math.abs(errorYaw) > 1.5) { 
                    outputYaw = pidyaw.calculate(botYaw, yawSet);
                } else {
                    outputYaw = pidyawi.calculate(botYaw, yawSet);
                }
                rAxis = outputYaw;
                break;
            case FREE:
                yawSet = 0;
                rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
                break;
            default: 
                yawSet = 0;
                rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
                break;
        }
        

        double yAxisDeadzoned = 0;
        double xAxisDeadzoned = 0;
        double yAxis = -controller.getRawAxis(translationAxis);//*Constants.Swerve.translationMultiplier);
        double xAxis = -controller.getRawAxis(strafeAxis);//*Constants.Swerve.translationMultiplier);

        yAxisDeadzoned = (Math.abs(yAxis) < Constants.OperatorConstants.joystickDeadzone) ? 0 : s_Swerve.map(Math.abs(yAxis), Constants.OperatorConstants.joystickDeadzone, 1.0, 0.0, 1.0);
        yAxisDeadzoned = yAxis >= 0.0 ? yAxisDeadzoned : -yAxisDeadzoned;
        yAxisDeadzoned = yAxisDeadzoned * yAxisDeadzoned; //(Math.cos(Math.PI*(yAxisDeadzoned + 1.0d)/2.0d)) + 0.5d;
        yAxisDeadzoned = yAxis >= 0.0 ? yAxisDeadzoned : -yAxisDeadzoned; 

        xAxisDeadzoned = (Math.abs(xAxis) < Constants.OperatorConstants.joystickDeadzone) ? 0 : s_Swerve.map(Math.abs(xAxis), Constants.OperatorConstants.joystickDeadzone, 1.0, 0.0, 1.0);
        xAxisDeadzoned = xAxis >= 0.0 ? xAxisDeadzoned : -xAxisDeadzoned;
        xAxisDeadzoned = xAxisDeadzoned * xAxisDeadzoned; //(Math.cos(Math.PI*(xAxisDeadzoned + 1.0d)/2.0d)) + 0.5d;
        xAxisDeadzoned = xAxis >= 0.0 ? xAxisDeadzoned : -xAxisDeadzoned;
        yAxisDeadzoned = yAxisDeadzoned > 1 ? 1 : yAxisDeadzoned;
        xAxisDeadzoned = xAxisDeadzoned > 1 ? 1 : xAxisDeadzoned;

        translation = new Translation2d(yAxisDeadzoned, xAxisDeadzoned).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity * GlobalVariables.Swerve.rotationMultiplier;
        if (s_Swerve.headingState == HeadingState.AIMED && Limelight.limelightshooter.HasTarget() == 1){
            rotation = pidyaw2.calculate(Limelight.limelightshooter.limelightTable.getEntry("tx").getDouble(0), 0);
        } else if (s_Swerve.headingState == HeadingState.AIMED && Limelight.limelightshooter.HasTarget() != 1){
            rAxis = -controller.getRawAxis(rotationAxis)*Constants.Swerve.rotationMultiplier;
            rotation = rAxis * Constants.Swerve.maxAngularVelocity * GlobalVariables.Swerve.rotationMultiplier;
        }
        
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

        // SmartDashboard.putNumber("yawNominal", botYaw);
        // SmartDashboard.putNumber("yawSet", yawSet);
        // SmartDashboard.putNumber("yawOutput", outputYaw);
        // SmartDashboard.putNumber("yawError", errorYaw);
        GlobalVariables.Swerve.translationX = yAxisDeadzoned*Constants.Swerve.maxSpeed;
        GlobalVariables.Swerve.translationY = xAxisDeadzoned*Constants.Swerve.maxSpeed;

        SmartDashboard.putNumber("translationX", GlobalVariables.Swerve.translationX);
        SmartDashboard.putNumber("translationY", GlobalVariables.Swerve.translationY);
    }
}