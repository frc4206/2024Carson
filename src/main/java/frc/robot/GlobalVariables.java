package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GlobalVariables {
    public static Alliance alliance;
    public static double distanceToSpeaker = 0;
    public static double desiredVelo = 0;
    public static double desiredAngle = 0;

    public static boolean intakingPiece = false;
    public static boolean pieceReady = false;
    public static boolean ampReady = false;

    public static double ampCounter = 2;
    public static double veloCounter = 2;
    public static double climbCounter = 2;

    public static double topVelo = 0.52;
    public static double bottomVelo = 0.28;
    
    public static double teleopTimeStart = 0;
    public static double teleopTimeElapsed = 0;
}