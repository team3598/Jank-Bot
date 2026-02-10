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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem intake = new IntakeSubsystem();
    private TurretSubsystem turret = new TurretSubsystem();
    private TowerAlignment alignment = new TowerAlignment();
    private final PoseSubsystem PoseSubsystem = new PoseSubsystem(drivetrain);
    private TurretCalibrationCommand turretCalibrationCommand = new TurretCalibrationCommand(turret, PoseSubsystem);


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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
       drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(
            turret.runEnd(
                () -> {
                    Translation2d realHub = turret.hubPosition; 
                    ChassisSpeeds robotVel = drivetrain.getFieldRelativeSpeed();
                    Pose2d robotPose = PoseSubsystem.getCurrentPose();                    
            
                    Translation2d virtualHub = turret.getCorrectedTargetPosition(realHub, robotVel, PoseSubsystem);
                    double virtualDist = robotPose.getTranslation().getDistance(virtualHub);

                    turret.setShooterVelocity(turret.m_shooterSpeedMap.get(virtualDist));
                    turret.setHoodAngle(turret.m_hoodAngleMap.get(virtualDist));

                    double dx = virtualHub.getX() - robotPose.getX();
                    double dy = virtualHub.getY() - robotPose.getY();

                    Rotation2d angleToTarget = new Rotation2d(dx, dy);
                    Rotation2d targetAngleRelative = angleToTarget.minus(robotPose.getRotation());
            
                    turret.moveTurretAngle(-targetAngleRelative.getDegrees() / 360.0);
            
                    if (turret.isShooterAtSpeed(turret.m_shooterSpeedMap.get(virtualDist))) {
                        turret.setFeederVelocity(80);
                        turret.setHopperSpeed(35);
                    }
                },
                () -> {
                    turret.stopMotors();
                    turret.setHoodAngle(-0.05); // Stow hood
                    intake.setIntakeVelocity(0);
                }
            )
        );
        joystick.y().whileTrue(intake.runIntakeCommand(30.0));
        joystick.x().whileTrue(
            turret.runEnd(
                () -> turret.setHopperSpeed(20),
                () -> turret.setHopperSpeed(0))
        );

        joystick.povLeft().whileTrue(turret.goToAngle(-90));
        joystick.povRight().whileTrue(turret.goToAngle(90));
        //joystick.povDown().whileTrue(turret.goToAngle(0));
        
        joystick.povUp().onTrue(intake.setIntakeVerticalPosition(-7.80));
        joystick.povDown().onTrue(intake.setIntakeVerticalPosition(0.05));


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

        /*joystick.povUp().whileTrue(
            turret.runEnd(
                () -> turret.setHoodAngle(turretCalibrationCommand.hoodTunerNumber),
                () -> turret.stopMotors()
                )
        );*/

        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //drivetrain.registerTelemetry(logger::telemeterize);
        System.out.println(
          "Hood Angle: " + turret.m_hoodAngleMap.get(PoseSubsystem.getDistToTarget(turret.hubPosition)) 
        + "Shooter Power: " + turret.m_hoodAngleMap.get(PoseSubsystem.getDistToTarget(turret.hubPosition)));
            
    }

 
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
