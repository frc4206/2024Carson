package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightCameraClass {

	//Initalize limelight
	public String limelightName;
	public NetworkTable limelightTable;
	public int camID;
	public boolean init = false;
	public double startTime;
	public double currTime;


	//April tag variables
	public double[] aprilTagResult = {0,0,0};
	public double[] fieldResult ={0,0,0};


	// Ai variables

	// how many degrees back is your limelight rotated from perfectly vertical?
	double limelightMountAngleDegrees = -25.0;

	// distance from the center of the Limelight lens to the floor
	double limelightLensHeightInches = 24.5;
	
	// distance from the target to the floor
	double goalHeightInches = 0;

	double distanceToTarget = 150;


	public LimelightCameraClass(int mcamID, String mlimelightName, double angle, double height, double targetHeight) {
		camID = mcamID;
		limelightName = mlimelightName;
		limelightMountAngleDegrees = angle;
		limelightLensHeightInches = height;
		goalHeightInches = targetHeight;

		limelightTable =  NetworkTableInstance.getDefault().getTable(limelightName);
	}


	public double HasTarget() {
		return limelightTable.getEntry("tv").getDouble(0);
	}
	public void ChangePipelines(int pipeline) {
			limelightTable.getEntry("pipeline").setNumber(pipeline);
		}

		public double GetPipeline() {
			return limelightTable.getEntry("pipeline").getDouble(0);
		}

	public double GetDistanceToGamePiece() {
		NetworkTableEntry ty = limelightTable.getEntry("ty");
		double targetOffsetAngle_Vertical = ty.getDouble(0.0);

		double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
		double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
		double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

		double distancemeters = distanceFromLimelightToGoalInches/39.37;

		return distancemeters;
	}

	public double[] GetGamePiecePosition(double[] OdometryArray, double angle) {
			double[] gamePiecePos = {0,0};

			angle %= 360;
			angle = (angle < 0) ? 360 + angle: angle;

			double limelightXangle = limelightTable.getEntry("tx").getDouble(0);

			//limelightXangle = (limelightXangle < 0) ? 360 + limelightXangle : limelightXangle;
			double finalAngle = angle + limelightXangle;

			

			gamePiecePos[0] = OdometryArray[0] + SmartDashboard.getNumber(limelightName + " ai distance: ", 0)* Math.cos(finalAngle * (3.14159 / 180.0));
			gamePiecePos[1] = OdometryArray[1] + SmartDashboard.getNumber(limelightName + " ai distance: ", 0)* Math.sin(finalAngle * (3.14159 / 180.0)); 

			SmartDashboard.putNumber(limelightName + "  X",  SmartDashboard.getNumber(limelightName + " ai distance: ", 0)* Math.cos(finalAngle*(3.14159 / 180.0)));
			SmartDashboard.putNumber(limelightName + " Y",SmartDashboard.getNumber(limelightName + " ai distance: ", 0)* Math.sin(finalAngle*(3.14159 / 180.0)));


			SmartDashboard.putNumber(limelightName + " final angle", finalAngle);
			SmartDashboard.putNumberArray(limelightName + " game peice array", gamePiecePos);

			return gamePiecePos;
	}

	public void UpdateResults() {
		//Updates the current pipleines calcuations
		
		if (limelightTable.getEntry("pipeline").getDouble(0) == 2) {
		if (HasTarget() == 1) {
			if (!init) {
			startTime = Timer.getFPGATimestamp();
			init = true;
			} else if (init) {
			currTime = Timer.getFPGATimestamp();
			}
			if (currTime > 1) {
			aprilTagResult = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
			aprilTagResult[2] *= -1;
			fieldResult = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
			}
			
		} else {
			init = false;
		}
			
		} else if (limelightTable.getEntry("pipeline").getDouble(0) == 1) {
			SmartDashboard.putNumber(limelightName + " ai distance: ", GetDistanceToGamePiece());

		}

	}
}
