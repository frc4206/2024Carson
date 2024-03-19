package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GlobalVariables {
    public static Alliance alliance;

    public static class Conveyor{
        public static boolean beamBroken = false;
    }

    public static class Shooter{
        public static double ampCounter = 2;
        public static double veloCounter = 2;
        public static double climbCounter = 2;
        public static boolean toAmpVelo = false;

        public static double topVelo = 0;
        public static double bottomVelo = 0;
    }

    public static class Pivot{
        public static double desiredPosition = 0;
    }

    public static class Position{
        public static double distanceToSpeaker = 0;
    }

    public static class Swerve{
        public static double rotationMultiplier = 1;

        public static double translationX = 0;
        public static double translationY = 0;
    }

    public static class Timing{
        public static double teleopTimeStart = 0;
        public static double teleopTimeElapsed = 0;
    }
}