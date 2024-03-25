// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.HeadingState;

public class TeleopSwerve extends Command {
  private SwerveSubsystem m_swerve;
  private XboxController m_controller;

  private double[] driverStickData = new double[3];

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(Constants.Swerve.maxTranslationVelocity * Constants.OperatorConstants.joystickDeadzone)
    .withRotationalDeadband(Constants.Swerve.maxRotationalVelocity * Constants.OperatorConstants.joystickDeadzone)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage
  );

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(0)
    .withRotationalDeadband(0)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage
  );

  private double angleFromTag = 0;
  private double botYaw = 0;
  private double yawSet = 0;


  public TeleopSwerve(SwerveSubsystem swerve, XboxController controller) {
    m_controller = controller;
    m_swerve = swerve;
    addRequirements(m_swerve);
  }

  private double map(double val, double inMin, double inMax, double outMin, double outMax) {
    return ((val-inMin)*(outMax-outMin)
        /(inMax-inMin))
        +outMin;
  }

  private double[] getDriverSticks(XboxController controller){
    double[] driverSticks = new double[3];
    double axisZero = -controller.getLeftY();
    double axisOne = -controller.getLeftX();
    double axisTwo = -controller.getRightX();

    driverSticks[0] = (Math.abs(axisZero) < Constants.OperatorConstants.joystickDeadzone) ? 0 : map(Math.abs(axisZero), Constants.OperatorConstants.joystickDeadzone, 1.0, 0.0, 1.0);
    driverSticks[0] = axisZero >= 0.0 ? driverSticks[0] : -driverSticks[0];
    driverSticks[0] = driverSticks[0] * driverSticks[0];
    driverSticks[0] = axisZero >= 0.0 ? driverSticks[0] : -driverSticks[0];
    driverSticks[0] = driverSticks[0] > 1 ? 1 : driverSticks[0];

    driverSticks[1] = (Math.abs(axisOne) < Constants.OperatorConstants.joystickDeadzone) ? 0 : map(Math.abs(axisOne), Constants.OperatorConstants.joystickDeadzone, 1.0, 0.0, 1.0);
    driverSticks[1] = axisOne >= 0.0 ? driverSticks[1] : -driverSticks[1];
    driverSticks[1] = driverSticks[1] * driverSticks[1];
    driverSticks[1] = axisOne >= 0.0 ? driverSticks[1] : -driverSticks[1];
    driverSticks[1] = driverSticks[1] > 1 ? 1 : driverSticks[1];

    driverSticks[2] = axisTwo;
    return driverSticks;
  }



  @Override
  public void execute() {
    botYaw = m_swerve.getState().Pose.getRotation().getDegrees();

    switch (m_swerve.headingState) {
      case AIMED:
        angleFromTag = Limelight.limelightshooter.limelightTable.getEntry("tx").getDouble(0);
        yawSet = botYaw - angleFromTag;
      case AMPED:
        if (GlobalVariables.alliance == Alliance.Blue) {
          if (botYaw > 0 && botYaw < 270) {
            yawSet = 90;
          } else {
            yawSet = -90;
          }
        } else if (GlobalVariables.alliance == Alliance.Red) {
          if (botYaw > 90 && botYaw < 360) {
            yawSet = 270;
          } else {
            yawSet = -90;
          }
        }
      case BACKWARD:
        if (botYaw > 0 && botYaw < 180) {
          yawSet = 0;
        } else {
          yawSet = 360;
        }
      case PICKUP:
        if (GlobalVariables.alliance == Alliance.Blue) {
          if (botYaw > 0 && botYaw < 180) {
            yawSet = -60;
          } else {
            yawSet = 300;
          }
        } else if (GlobalVariables.alliance == Alliance.Red) {
          if (botYaw > 0 && botYaw < 180) {
            yawSet = 60;
          } else {
            yawSet = -300;
          }
        }
      case FREE:
        break;
      default:
        break;
    }

    driverStickData = getDriverSticks(m_controller);

    if (m_swerve.headingState == HeadingState.FREE || (m_swerve.headingState == HeadingState.AIMED && Limelight.limelightshooter.HasTarget() != 1)){
      m_swerve.applyRequest(
        () -> drive.withVelocityX(driverStickData[0] * Constants.Swerve.maxTranslationVelocity)
                   .withVelocityY(driverStickData[1] * Constants.Swerve.maxTranslationVelocity)
                   .withRotationalRate(driverStickData[2] * Constants.Swerve.maxRotationalVelocity)
      );
    } else {
      m_swerve.applyRequest(
        () -> driveAtAngle.withVelocityX(driverStickData[0] * Constants.Swerve.maxTranslationVelocity)
                          .withVelocityY(driverStickData[1] * Constants.Swerve.maxTranslationVelocity)
                          .withTargetDirection(new Rotation2d(yawSet))
      );  
    }
  }
}