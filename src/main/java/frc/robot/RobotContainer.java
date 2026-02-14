// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.commands.Autos.TowerAlignment;
import frc.robot.commands.TurretCalibrationCommand;
import frc.robot.LimelightHelpers;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //private final CommandXboxController ps5Controller = new CommandXboxController(0);
    private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem intake = new IntakeSubsystem();
    private TurretSubsystem turret = new TurretSubsystem();
    private TowerAlignment alignment = new TowerAlignment();
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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
       drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double currentMaxSpeed;
                currentMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                /* 
                if (isShooting) {
                    currentMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3.0;
                } else {
                    currentMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                }*/ //for testing, i had a crappy controller before but now we got fresh ones :p
 
             return drive.withVelocityX(-ps5Controller.getLeftY() * currentMaxSpeed)
                    .withVelocityY(-ps5Controller.getLeftX() * currentMaxSpeed) 
                    .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate);
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        turret.setDefaultCommand(
            turret.run(() -> {
                if (isAiming) {
                    runAiming();
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
                    double virtualDist = runAiming(); 

                    double targetSpeed = turret.m_shooterSpeedMap.get(virtualDist);
                    double targetAngle = turret.m_hoodAngleMap.get(virtualDist);

                    //turret.setShooterVelocity(targetSpeed);
                    //turret.setHoodAngle(targetAngle);

                    //turret.setShooterVelocity(turret.m_shooterSpeedMap.get(virtualDist));
                    turret.setShooterVelocity(100);
                    turret.setHoodAngle(turret.m_hoodAngleMap.get(virtualDist));
            
                    if (turret.isShooterAtSpeed(turret.m_shooterSpeedMap.get(virtualDist))) {
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

        //ps5Controller.povLeft().whileTrue(turret.goToAngle(-90));
        //ps5Controller.povRight().whileTrue(turret.goToAngle(90));
        //ps5Controller.povDown().whileTrue(turret.goToAngle(0));
        
        //ps5Controller.povUp().onTrue(intake.setIntakeVerticalPosition(-7.80));
        //ps5Controller.povDown().onTrue(intake.setIntakeVerticalPosition(0.05));


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

        /*ps5Controller.povUp().whileTrue(
            turret.runEnd(
                () -> turret.setHoodAngle(turretCalibrationCommand.hoodTunerNumber),
                () -> turret.stopMotors()
                )
        );*/

        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*ps5Controller.back().and(ps5Controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        ps5Controller.back().and(ps5Controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        ps5Controller.start().and(ps5Controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        ps5Controller.start().and(ps5Controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

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

        //drivetrain.registerTelemetry(logger::telemeterize);
        System.out.println(
          "Hood Angle: " + turret.m_hoodAngleMap.get(PoseSubsystem.getDistToTarget(turret.hubPosition)) 
        + "Shooter Power: " + turret.m_hoodAngleMap.get(PoseSubsystem.getDistToTarget(turret.hubPosition)));
            
    }
    
    private double runAiming() {
        double realVisionDist = PoseSubsystem.getDistToTarget(turret.hubPosition);
        Pose2d robotPose = PoseSubsystem.getCurrentPose();
        ChassisSpeeds robotVel = drivetrain.getFieldRelativeSpeed();

        double t = turret.calculateTimeOfFlight(realVisionDist);

        double shiftX = -robotVel.vxMetersPerSecond * t;
        double shiftY = -robotVel.vyMetersPerSecond * t;
    
        Translation2d virtualHub = new Translation2d(
            turret.hubPosition.getX() + shiftX,
            turret.hubPosition.getY() + shiftY
        );

        Translation2d robotToTarget = turret.hubPosition.minus(robotPose.getTranslation());
        Rotation2d angleToTarget = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
        double velTowardsTarget = (robotVel.vxMetersPerSecond * angleToTarget.getCos()) + 
                              (robotVel.vyMetersPerSecond * angleToTarget.getSin());

        double virtualDist = realVisionDist - (velTowardsTarget * t);

        if (virtualDist < 1.0) virtualDist = 1.0; 

        turret.setHoodAngle(turret.m_hoodAngleMap.get(virtualDist));
    
        double dx = virtualHub.getX() - robotPose.getX();
        double dy = virtualHub.getY() - robotPose.getY();
        Rotation2d targetRot = new Rotation2d(dx, dy);
        Rotation2d relativeRot = targetRot.minus(robotPose.getRotation());
        double aimAngle = Math.IEEEremainder(relativeRot.getDegrees(), 360.0);
        double robotSpinRPS = drivetrain.getState().Speeds.omegaRadiansPerSecond / (2 * Math.PI);
        double counteractedSpinRPS = -robotSpinRPS; //for later

        turret.moveTurretAngle(-aimAngle / 360.0);
        //System.out.println("Robot Speed (m/s): " + robotVel.vxMetersPerSecond);
        return virtualDist; 
    }


    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
