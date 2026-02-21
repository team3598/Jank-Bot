package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class FMSTracker {
    private static String currentGameData = "";

    public static Alliance getActiveAlliance() {
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        var currentTime = DriverStation.getMatchTime();

        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        Alliance initialAlliance = DriverStation.getGameSpecificMessage().charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        if (currentTime >= 130 || currentTime < 30) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }
        else if (currentTime >= 105 || (currentTime < 80 && currentTime >= 55)) {
            return initialAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
        }
        else {
            return initialAlliance;
        }
    }

    public static boolean isAllianceHubActive() {
        Alliance activeAlliance = getActiveAlliance();
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (activeAlliance == null) {
            return false;
        }
        
        return activeAlliance == teamAlliance;
    }



}