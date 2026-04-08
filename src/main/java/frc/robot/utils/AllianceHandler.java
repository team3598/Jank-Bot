package frc.robot.utils;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class AllianceHandler {
    private static String currentGameData = "";

    public static Alliance getActiveAlliance() {
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        var currentTime = Timer.getFPGATimestamp();

        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        Alliance initialAlliance = DriverStation.getGameSpecificMessage().charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        if (currentTime >= currentTime + 130 || currentTime < currentTime + 30) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }
        else if (currentTime >= currentTime + 105 || (currentTime < currentTime + 80 && currentTime >= currentTime + 55)) {
            return initialAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
        }
        else {
            return initialAlliance;
        }
    }

    public static Alliance checkAllianceSide() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);    
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