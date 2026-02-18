// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    private TurretCalibrationCommand turretCalibrationCommand = new TurretCalibrationCommand(turret, PoseSubsystem);

    public RobotContainer() {
        configureBindings();
        NamedCommands.registerCommand("IntakeOn", intake.beginIntakeCommand());
        NamedCommands.registerCommand("IntakeOff", intake.endIntakeCommand());
        NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());
        

 
        autoChooser = AutoBuilder.buildAutoChooser("intaketest");
        

        SmartDashboard.putData("Auto Mode", autoChooser);
        //turretCalibrationCommand.ignoringDisable(true).schedule();
        FollowPathCommand.warmupCommand().schedule();
    }
    
    private void configureBindings() {
       drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double currentMaxSpeed = MaxSpeed;
                
                if (isShooting) {
                    currentMaxSpeed = MaxSpeed / 3.0; 
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

        /*turret.setDefaultCommand(
            turret.run(() -> {
                if (isAiming) {
                    turret.autoAim(PoseSubsystem.getCurrentPose(), drivetrain.getFieldRelativeSpeed());
                } else {
                    turret.stopMotors();
                    turret.setHoodAngle(-0.05);
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
                        drivetrain.getFieldRelativeSpeed()
                    ); 

                    double targetSpeed = turret.m_shooterSpeedMap.get(virtualDist);
                    //hood's already set in turret subsystem

                    turret.setShooterVelocity(targetSpeed);
            
                    if (turret.isShooterAtSpeed(targetSpeed)) {
                        turret.setFeederVelocity(80);
                        turret.setHopperSpeed(35);
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

        ps5Controller.L2().whileTrue(intake.runIntakeCommand(30.0));
        ps5Controller.square().whileTrue(
            turret.runEnd(
                () -> turret.setHopperSpeed(20),
                () -> turret.setHopperSpeed(0))
        );

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
    
        ps5Controller.povUp().onTrue(
            Commands.run(
                () -> intake.setIntakeVerticalPosition(7.95))
        );

        ps5Controller.povDown().onTrue(
            Commands.run(
                () -> intake.setIntakeVerticalPosition(0.00))
        );*/

        ps5Controller.L1().onTrue(Commands.runOnce(SignalLogger::start));
ps5Controller.R1().onTrue(Commands.runOnce(SignalLogger::stop));

/*
 * Joystick Y = quasistatic forward
 * Joystick A = quasistatic reverse
 * Joystick B = dynamic forward
 * Joystick X = dyanmic reverse
 */
ps5Controller.triangle().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
ps5Controller.cross().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
ps5Controller.circle().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
ps5Controller.square().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //ps5Controller.povLeft().whileTrue(turret.goToAngle(-90));
        //ps5Controller.povRight().whileTrue(turret.goToAngle(90));
        //ps5Controller.povDown().whileTrue(turret.goToAngle(0));
    


        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(() -> LimelightHelpers.SetThrottle("limelight-bright", 100))
            .ignoringDisable(true) 
        );

        RobotModeTriggers.teleop().onTrue(
            new InstantCommand(() -> LimelightHelpers.SetThrottle("limelight-bright", 0))
            .ignoringDisable(true)
        );

        RobotModeTriggers.autonomous().onTrue(
            new InstantCommand(() -> LimelightHelpers.SetThrottle("limelight-bright", 0))
            .ignoringDisable(true)
        );  
    }
    
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}