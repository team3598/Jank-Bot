// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//unfinished
package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.vision.*;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class RegisterAutoCommands {
        final static Alignment alignment = new Alignment();
        final static IntakeSubsystem intake = new IntakeSubsystem();
        final static TurretSubsystem turret = new TurretSubsystem();
        final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        final static PoseSubsystem poseSubsystem = new PoseSubsystem(drivetrain);
        final static Translation2d hubPosition = new Translation2d(4.625, 4.035);
        final static boolean nearTrench = false;

    public RegisterAutoCommands() {
        
    }

    public static void registerNamedCommands()
    {
        NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());
        NamedCommands.registerCommand("RunIntakeCommand", Commands.startEnd(
            () -> intake.intakeDownAndIntakeCommand(() -> drivetrain.getFieldRelativeSpeed()),
            () -> intake.setIntakeVelocity(0)
        ));
        NamedCommands.registerCommand("IntakeOn", intake.beginIntakeCommand(() -> drivetrain.getFieldRelativeSpeed()));
        NamedCommands.registerCommand("IntakeOff", intake.endIntakeCommand());
        NamedCommands.registerCommand("IntakeDown", intake.intakeDown());
        NamedCommands.registerCommand("AgitateIntake", intake.intakeAgitate());
        NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());
        //NamedCommands.registerCommand("ShootAtHub", turret.getAutoAimAndShootCommand(poseSubsystem, drivetrain, hubPosition, nearTrench));

        NamedCommands.registerCommand("StopShooting", turret.runOnce(() -> {
            turret.stopMotors();
            turret.stopFeeding();
            turret.setHoodPosition(-0.3);
        }));

    }
}
