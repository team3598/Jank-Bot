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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem intake = new IntakeSubsystem();
    private TurretSubsystem turret = new TurretSubsystem();
    private Alignment alignment = new Alignment();
    private final PoseSubsystem PoseSubsystem = new PoseSubsystem(drivetrain);
    private boolean isAiming = false;
    private boolean isShooting = false;
    private double unjammingPower = 1.0;
    private double jamStartTime = 0.0;
    private TurretCalibrationCommand turretCalibrationCommand = new TurretCalibrationCommand(turret, PoseSubsystem);

    public Translation2d hubPosition = new Translation2d(4.625, 4.035);


    public RobotContainer() {
        configureBindings();
        NamedCommands.registerCommand("IntakeOn", intake.beginIntakeCommand());
        NamedCommands.registerCommand("IntakeOff", intake.endIntakeCommand());
        NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());
        
        autoChooser = AutoBuilder.buildAutoChooser("intaketest");
        

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
                    turret.autoAim(PoseSubsystem.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), hubPosition);
                } else {
                    turret.stopMotors();
                    turret.setHoodAngle(0);
                }
            })
        );

        ps5Controller.cross().onTrue(
            Commands.runOnce(() -> {
                isAiming = !isAiming;
            })
        );

        ps5Controller.R2().whileTrue(
            turret.runEnd(
                () -> {
                    ps5Controller.setRumble(RumbleType.kBothRumble, 1);
                    isShooting = true;

                    double virtualDist = turret.autoAim(
                        PoseSubsystem.getCurrentPose(), 
                        drivetrain.getFieldRelativeSpeed(),
                        hubPosition
                    ); 

                    double targetSpeed = turret.m_shooterSpeedMap.get(virtualDist); //hood's already set in turret subsystem

                    turret.setShooterVelocity(targetSpeed);
                    //turret.setShooterVelocity(turretCalibrationCommand.flywheelTunerNumber);
                    if (turret.isShooterAtSpeed(targetSpeed) && turret.isTurretAligned(1.5))
                    {   
                        try { Thread.sleep((long) 50); } catch (InterruptedException e) {} // so hopper catches up to speed
                        if (Math.abs(turret.getHopperSpeed()) <= 10.0)
                        {   
                            if (jamStartTime == 0) jamStartTime = PoseSubsystem.timeMS;
                            double timeJammed = PoseSubsystem.timeMS - jamStartTime;
                            unjammingPower = -20 - (Math.min(1, timeJammed * 1.1));
                            turret.setHopperVelocity(unjammingPower); 
                            //double targetTime = PoseSubsystem.timeMS + 40;
                        } else {
                            turret.setHoodAngle(0);
                            turret.setFeederVelocity(90);
                            turret.setHopperVelocity(50);
                        }
                    }
                },
                () -> {
                    ps5Controller.setRumble(RumbleType.kBothRumble, 0);
                    isShooting = false;
                    turret.stopMotors();
                    turret.setHoodAngle(-0.3);
                    intake.setIntakeVelocity(0);
                }
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
        
        ps5Controller.povUp().onTrue(intake.setIntakeVerticalPosition(0.05));
        ps5Controller.povDown().onTrue(intake.setIntakeVerticalPosition(-7.1));
        ps5Controller.povLeft().whileTrue(
            runEnd(
                () -> turret.setFeederVelocity(70),
                () -> turret.setFeederVelocity(0)
            ));

        ps5Controller.povRight().whileTrue(
            runEnd(
                () -> turret.setHopperVelocity(70),
                () -> turret.setHopperVelocity(0)
            ));


        //ps5Controller.povLeft().whileTrue(turret.goToAngle(-90));
        //ps5Controller.povRight().whileTrue(turret.goToAngle(90));
        //ps5Controller.povDown().whileTrue(turret.goToAngle(0));
    


        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(() -> {
                LimelightHelpers.SetThrottle("limelight-fleft", 100);
                LimelightHelpers.SetThrottle("limelight-fright", 100);
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