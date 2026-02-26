// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.commands.Autos.Alignment;
import frc.robot.commands.TurretCalibrationCommand;
import frc.robot.LimelightHelpers;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0);
    private final PS4Controller HIDController = new PS4Controller(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem intake = new IntakeSubsystem();
    private TurretSubsystem turret = new TurretSubsystem();
    private Alignment alignment = new Alignment();
    private final PoseSubsystem PoseSubsystem = new PoseSubsystem(drivetrain);
    private boolean isAiming = false;
    private boolean isShooting = false;
    private TurretCalibrationCommand turretCalibrationCommand = new TurretCalibrationCommand(turret, PoseSubsystem);
    private boolean nearTrench = false;
    public Translation2d hubPosition = new Translation2d(4.625, 4.035);


    public RobotContainer() {
        configureBindings();
        NamedCommands.registerCommand("IntakeOn", intake.beginIntakeCommand());
        NamedCommands.registerCommand("IntakeOff", intake.endIntakeCommand());
        NamedCommands.registerCommand("IntakeUp", intake.intakeUp());
        NamedCommands.registerCommand("IntakeDown", intake.intakeDown());
        NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());
        NamedCommands.registerCommand("ShootAtHub", turret.getAutoAimAndShootCommand(PoseSubsystem, drivetrain, hubPosition, nearTrench));

        NamedCommands.registerCommand("StopShooting", turret.runOnce(() -> {
            turret.stopMotors();
            turret.stopFeeding();
            turret.setHoodPosition(-0.3);
        }));

        autoChooser = AutoBuilder.buildAutoChooser("T1ShootNeutral");
        

        SmartDashboard.putData("Auto Mode", autoChooser);
        turretCalibrationCommand.ignoringDisable(true).schedule();
        FollowPathCommand.warmupCommand().schedule();
    }
    
    private void configureBindings() {
       drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double currentMaxSpeed = MaxSpeed;
                
                if (isShooting) {
                    currentMaxSpeed = MaxSpeed / 2.0; 
                }

                return drive.withVelocityX(-ps5Controller.getLeftY() * currentMaxSpeed)
                        .withVelocityY(-ps5Controller.getLeftX() * currentMaxSpeed) 
                        .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        turret.setDefaultCommand(
            turret.run(() -> {
                if (isAiming) {
                    turret.autoAim(PoseSubsystem.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), hubPosition, drivetrain);
                    if (PoseSubsystem.getCurrentPose().getX() >= 3.5 && PoseSubsystem.getCurrentPose().getX() <= 5.75) {
                        nearTrench = true;
                    } else {
                        nearTrench = false;
                    }
                } else {
                    turret.stopMotors();
                    turret.setHoodPosition(-0.3);
                }
            })
        );

        ps5Controller.cross().onTrue(
            Commands.runOnce(() -> {
                isAiming = !isAiming;
            })
        );

        ps5Controller.L3().whileTrue(
            turret.runOnce(
                () -> turret.setHopperVelocity(-30))
        );

        ps5Controller.R2().whileTrue(
            turret.getAutoAimAndShootCommand(PoseSubsystem, drivetrain, hubPosition, nearTrench)
            .alongWith(
                Commands.startEnd(
                    () -> {
                        isShooting = true;
                        HIDController.setRumble(RumbleType.kBothRumble, 1);
                        HIDController.setOutput(1, true);
                    },
                    () -> {
                        isShooting = false;
                        HIDController.setRumble(RumbleType.kBothRumble, 0);
                        HIDController.setOutput(0, false);
                    }
                )
            )
        );

        ps5Controller.L2().whileTrue(intake.runIntakeCommand(40.0));

        ps5Controller.L1().onTrue(
            Commands.either(
                alignment.toTrench1AS().andThen(alignment.toNeutralZoneFromT1()),
                alignment.toNZTrench1().andThen(alignment.toT1FromNZ()),
                () -> PoseSubsystem.getCurrentPose().getX() < 4.5
            )
        );

        ps5Controller.R1().onTrue(
            Commands.either(
                alignment.toTrench2AS().andThen(alignment.toNeutralZoneFromT2()),
                alignment.toNZTrench2().andThen(alignment.toT2FromNZ()),
                () -> PoseSubsystem.getCurrentPose().getX() < 4.5
            )
        );
        
        ps5Controller.povUp().onTrue(intake.setIntakeVerticalPosition(6.14));
        ps5Controller.povDown().onTrue(intake.setIntakeVerticalPosition(-0.05));
        
        ps5Controller.povLeft().whileTrue(
            turret.runEnd(
                () -> turret.setHoodPosition(13.0), 
                () -> turret.setHoodPosition(0)
                )
        );

        //ps5Controller.povRight().whileTrue(turret.goToAngle(90));
        //ps5Controller.povDown().whileTrue(turret.goToAngle(0));
    


        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(() -> {
                LimelightHelpers.SetThrottle("limelight-fleft", 200);
                LimelightHelpers.SetThrottle("limelight-fright", 200);
            })
            .ignoringDisable(true)
        );

        RobotModeTriggers.teleop().onTrue(
            new InstantCommand(() -> {
                LimelightHelpers.SetThrottle("limelight-fleft", 0);
                LimelightHelpers.SetThrottle("limelight-fright", 0);
            })
            .ignoringDisable(true) 
        );

        RobotModeTriggers.autonomous().onTrue(
            new InstantCommand(() -> {
                LimelightHelpers.SetThrottle("limelight-fleft", 0);
                LimelightHelpers.SetThrottle("limelight-fright", 0);
            })
            .ignoringDisable(true) 
        );  
    }
    
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}