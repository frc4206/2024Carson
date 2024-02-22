// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.LimelightCameraClass;
import frc.robot.LimelightCameraManagerClass;

public class Limelight extends SubsystemBase {
  public static LimelightCameraClass limelightshooter = new LimelightCameraClass(Constants.Limelight.limelightFrontCamID, "limelight-shooter", Constants.Limelight.limelightFrontAngle, Constants.Limelight.limelightFrontHeight, Constants.Limelight.limelightFrontTargetHeight);
  static LimelightCameraClass limelightleft = new LimelightCameraClass(Constants.Limelight.limelightLeftCamID, "limelight-left", Constants.Limelight.limelightLeftAngle, Constants.Limelight.limelightLeftHeight, Constants.Limelight.limelightLeftTargetHeight);
  static LimelightCameraClass limelightright = new LimelightCameraClass(Constants.Limelight.limelightRightCamID, "limelight-right", Constants.Limelight.limelightRightAngle, Constants.Limelight.limelightRightHeight, Constants.Limelight.limelightRightTargetHeight);

  static LimelightCameraClass[] limelightList = {limelightshooter, limelightleft, limelightright};

  public static LimelightCameraManagerClass limelightManger = new LimelightCameraManagerClass(limelightList);

  static SwerveSubsystem swerve = new SwerveSubsystem();

  public static boolean isenabled = false;
  boolean init = false;

  static int ctr;
  static double[] rawbotpose;
  static double[] botpose;

  public Limelight() {}

  public void ChangePipelines(int pipeline) {
    limelightManger.changeAllPipelines(pipeline);
  }

  public static double[] getFieldCordsTEST() {
    return limelightManger.pose;
  }



  public static boolean ATDetected(){
    return (NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("tid").getDouble(0) != -1.0);
  }
  
  public static boolean ATwithinDistance(){
    if (ATDetected()){
      rawbotpose = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose").getDoubleArray(new double[6]);
      if ((rawbotpose[0] - frc.robot.Constants.Limelight.fieldXOffset < 0.6) || (rawbotpose[1] - frc.robot.Constants.Limelight.fieldYOffset < 0.05)){
        return false;
      } if (rawbotpose[0] - frc.robot.Constants.Limelight.fieldXOffset > 4.1 && rawbotpose[0] - frc.robot.Constants.Limelight.fieldXOffset < 15.8){
        return false;
      } else if (rawbotpose[0] - frc.robot.Constants.Limelight.fieldXOffset < 4.1 || rawbotpose[0] - frc.robot.Constants.Limelight.fieldXOffset > 15.8){
        return true;
      }
    }
    return false;
  }

  public static double[] getBotPoseAdjusted(){
    ctr = 0;
    rawbotpose = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("botpose").getDoubleArray(new double[6]);
    botpose = new double[6];
    for (double posepoint : rawbotpose){
      if (ctr == 5) {
        break;
      } if (ctr==0) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.fieldXOffset;
          } else {
            if (ATwithinDistance()){
              botpose[ctr] = posepoint - frc.robot.Constants.Limelight.fieldXOffset;
            } else {
              posepoint = 0;
              botpose[ctr] = posepoint + frc.robot.Constants.Limelight.fieldXOffset;
            }
          }
      } else if (ctr==1) {
          if (posepoint == 0){
            botpose[ctr] = posepoint + frc.robot.Constants.Limelight.fieldYOffset;
          } else {
            if (ATwithinDistance()){
              botpose[ctr] = posepoint - frc.robot.Constants.Limelight.fieldYOffset;
            } else {
              posepoint = 0;
              botpose[ctr] = posepoint + frc.robot.Constants.Limelight.fieldYOffset;
            }
          }
      } else if (ctr==2) {
        botpose[ctr] = swerve.getNominalYaw();
      } else {
        botpose[ctr] = posepoint;
      }
      ctr+=1;
    }
    if (((botpose[0] != frc.robot.Constants.Limelight.fieldXOffset) && (botpose[1] != frc.robot.Constants.Limelight.fieldYOffset)) && ATwithinDistance()){
      swerve.resetOdometry(new Pose2d(new Translation2d(botpose[0], botpose[1]), swerve.getYaw()));
    } else {
      swerve.swerveOdometry.update(swerve.getYaw(), swerve.getModulePositions());
    }
    if (!ATDetected() || !ATwithinDistance()){
      botpose[0] = swerve.swerveOdometry.getPoseMeters().getX();
      botpose[1] = swerve.swerveOdometry.getPoseMeters().getY();
    }
    GlobalVariables.distanceToSpeaker = Math.sqrt(botpose[0]*botpose[0] + (botpose[1]-5.52)*(botpose[1]-5.52));
    SmartDashboard.putNumberArray("Field Pose", botpose);
    SmartDashboard.putNumber("DistToSpeaker", GlobalVariables.distanceToSpeaker);
    return botpose;
  }
  


  @Override
  public void periodic() {
    // limelightManger.Update();
    getBotPoseAdjusted();
  } 
}
