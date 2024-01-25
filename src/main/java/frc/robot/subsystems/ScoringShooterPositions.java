// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Figure out optimal lines to score from essential field positions as well as around our wing of the field */
public class ScoringShooterPositions {
    public String allianceColour;
    public String objectName;
    public double robotOrientationDegrees;
    public double robotXCoords;
    public double robotYCoords;

    private double xCoords;
    private double yCoords;

    public PivotSubsystem pivotSubsystem;

    // Some preset angles so that we don't have to worry about (x, y) coordinates all the time
    public enum PresetAngles {
        AMPLIFIER,
        SPEAKER, // Most reliable (a.k.a. Subwoofer)
        PODIUM,
        SOURCE,
        ORIGIN, // Just for context purposes
        WING // Where exactly is the wing?
    }

    public ScoringShooterPositions(PivotSubsystem pivot, String allyColour, String name, double robotDegrees, double robotX, double robotY) {
        pivot = pivotSubsystem;
        allyColour = allianceColour;
        name = objectName;
        robotDegrees = robotOrientationDegrees;
        robotX = robotXCoords;
        robotY = robotYCoords;
    }

    public double xCoord() {
        return xCoord();//might break IDK
    }

    public double getPresetCoordinates(PresetAngles angle) {
        // Origin position
        double xCoord = 0;
        double yCoord = 0;
        switch(angle) {
            case AMPLIFIER:
                xCoord = 27; // PLACEHOLDER VALUES!
                yCoord = 4;
                break;
            
        }
        return xCoord;
    }

}
