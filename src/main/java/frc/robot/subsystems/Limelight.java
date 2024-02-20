// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightCameraClass;
import frc.robot.LimelightCameraManagerClass;

public class Limelight extends SubsystemBase {
/** Creates a new Limelight. */

  public LimelightCameraClass limelightshooter = new LimelightCameraClass(Constants.Limelight.limelightFrontCamID, "limelight-shooter", Constants.Limelight.limelightFrontAngle, Constants.Limelight.limelightFrontHeight, Constants.Limelight.limelightFrontTargetHeight);
  LimelightCameraClass limelightleft = new LimelightCameraClass(Constants.Limelight.limelightLeftCamID, "limelight-left", Constants.Limelight.limelightLeftAngle, Constants.Limelight.limelightLeftHeight, Constants.Limelight.limelightLeftTargetHeight);
  LimelightCameraClass limelightright = new LimelightCameraClass(Constants.Limelight.limelightRightCamID, "limelight-right", Constants.Limelight.limelightRightAngle, Constants.Limelight.limelightRightHeight, Constants.Limelight.limelightRightTargetHeight);

  LimelightCameraClass[] limelightList = {limelightshooter, limelightleft, limelightright};

  public LimelightCameraManagerClass limelightManger = new LimelightCameraManagerClass(limelightList);

  public static boolean isenabled = false;
  boolean init = false;

  public Limelight() {}

  public void ChangePipelines(int pipeline) {
    limelightManger.changeAllPipelines(pipeline);
  }

  public double[] getFieldCordsTEST() {
    return limelightManger.pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelightManger.Update();

      //if (!isenabled) {
      //  ChangePipelines(0);
      //  init= true;
      //}
  } 
}
