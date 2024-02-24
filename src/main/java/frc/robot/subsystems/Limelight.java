// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

	@Override
	public void periodic() {
		limelightManger.Update();
	} 
}
