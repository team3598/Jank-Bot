package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.utils.*;

public class Alignment{
    private final Pose2d blueTowerPosition = new Pose2d(1.6, 3.75, Rotation2d.fromDegrees(0));
    private final Pose2d blueTrench1 = new Pose2d(3.500, 7.4, Rotation2d.fromDegrees(0));
    private final Pose2d blueTrench2 = new Pose2d(3.375, .7, Rotation2d.fromDegrees(0));
    private final Pose2d blueNZtrench1 = new Pose2d(5.9, 7.4, Rotation2d.fromDegrees(0));
    private final Pose2d blueNZtrench2 = new Pose2d(5.9, .7, Rotation2d.fromDegrees(0));

    private final Pose2d redTowerPosition = new Pose2d(1.6, 3.75, Rotation2d.fromDegrees(0));
    private final Pose2d redTrench1 = new Pose2d(13.500, 7.4, Rotation2d.fromDegrees(0));
    private final Pose2d redTrench2 = new Pose2d(13.375, .7, Rotation2d.fromDegrees(0));
    private final Pose2d redNZtrench1 = new Pose2d(10.9, 7.4, Rotation2d.fromDegrees(0));
    private final Pose2d redNZtrench2 = new Pose2d(10.9, .7, Rotation2d.fromDegrees(0));

    /*private Pose2d towerPosition;
    private Pose2d trench1;
    private Pose2d trench2;
    private Pose2d NZtrench1;
    private Pose2d NZtrench2;*/

    private final PathConstraints constraints = new PathConstraints(3, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    //insert more positions for important places here
    
    public Command alignToTower() {
        return Commands.defer(() -> {
            Pose2d target = (AllianceHandler.checkAllianceSide() == Alliance.Red) ? redTowerPosition : blueTowerPosition;
            return AutoBuilder.pathfindToPose(target, constraints, 0.0);
        }, Set.of());
    }

   public Command toTrench1AS() { 
        return Commands.defer(() -> {
            Pose2d target = (AllianceHandler.checkAllianceSide() == Alliance.Red) ? redTrench1 : blueTrench1;
            return AutoBuilder.pathfindToPose(target, constraints, 0.0);
        }, Set.of());
    } 

    public Command toNZTrench1() { 
        return Commands.defer(() -> {
                Pose2d target = (AllianceHandler.checkAllianceSide() == Alliance.Red) ? redNZtrench1 : blueNZtrench1;
                return AutoBuilder.pathfindToPose(target, constraints, 0.0);
            }, Set.of());
    }       

    public Command toNZTrench2() { 
        return Commands.defer(() -> {
                Pose2d target = (AllianceHandler.checkAllianceSide() == Alliance.Red) ? redNZtrench2 : blueNZtrench2;
                return AutoBuilder.pathfindToPose(target, constraints, 0.0);
            }, Set.of());
    }       

    public Command toTrench2AS() { 
        return Commands.defer(() -> {
                Pose2d target = (AllianceHandler.checkAllianceSide() == Alliance.Red) ? redTrench2 : blueTrench2;
                return AutoBuilder.pathfindToPose(target, constraints, 0.0);
            }, Set.of());
   }        

  public Command toNeutralZoneFromT1() { 
    try{
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Trench 1 To Neutral");

            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

   public Command T1ShootNeutral(){
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("T1ShootNeutral");
            return AutoBuilder.followPath(path);
        } catch (Exception e){
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
            }
        }

  public Command toNeutralZoneFromT2() { 
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Trench 2 to Neutral");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }

}

  public Command toT1FromNZ() { 
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Neutral to Trench 1");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
   }

}

  public Command toT2FromNZ() { 
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Neutral to Trench 2");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
   }

}

}