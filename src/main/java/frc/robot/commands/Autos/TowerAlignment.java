package frc.robot.commands.Autos;

import frc.robot.subsystems.PoseSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;



public class TowerAlignment{
    private final Pose2d towerPosition = new Pose2d(1.6, 3.75, Rotation2d.fromDegrees(0));
    private final Pose2d trench1 = new Pose2d(3.375, 7.4, Rotation2d.fromDegrees(0));
    private final Pose2d trench2 = new Pose2d(3.375, .7, Rotation2d.fromDegrees(0));
    private final Pose2d NZtrench1 = new Pose2d(5.9, 7.4, Rotation2d.fromDegrees(0));
    private final Pose2d NZtrench2 = new Pose2d(3.375, .7, Rotation2d.fromDegrees(0));
    private final PathConstraints constraints = new PathConstraints(3, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    //insert more positions for important places here


    public Command alignToTower() {
        final Command towerAlignCommand = AutoBuilder.pathfindToPose(
            towerPosition, 
            constraints,
            0.0);
        return towerAlignCommand;
    }

   public Command toTrench1AS() { 
    final Command driveTrench1Command = AutoBuilder.pathfindToPose(
        trench1,
        constraints,
    0.0);
    return driveTrench1Command;
   } 

          public Command toNZTrench1() { 
    final Command driveTrench1NZCommand = AutoBuilder.pathfindToPose(
        NZtrench1,
        constraints,
    0.0);
    return driveTrench1NZCommand;
   }       

       public Command toNZTrench2() { 
    final Command driveTrench2NZCommand = AutoBuilder.pathfindToPose(
        NZtrench2,
        constraints,
    0.0);
    return driveTrench2NZCommand;
   }       

    public Command toTrench2AS() { 
    final Command driveTrench2Command = AutoBuilder.pathfindToPose(
        trench2,
        constraints,
    0.0);
    return driveTrench2Command;
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