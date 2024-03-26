package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GlobalVariables {
    public static Alliance alliance;

    public static class AmpBar{
        public static double ampBarPosition = 0;
        public static boolean ampBarAtLimitSwitch = false;
        public static boolean ampBarAtAmp = false;
        public static boolean ampBarAtZero = false;
    }

    public static class Conveyor{
        public static double conveyorPosition = 0;
        public static double conveyorVelocity = 0;
        public static boolean beamBroken = false;
    }

    public static class Flywheel{
        public static double ampCounter = 2;
        public static double veloCounter = 2;
        public static double climbCounter = 2;
        public static boolean toAmpVelo = false;
        public static boolean toPassVelo = false;

        public static double topVelo = 0;
        public static double bottomVelo = 0;
    }

    public static class Pivot{
        public static double desiredPosition = 0;
        public static double pivotPosition = 0;
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