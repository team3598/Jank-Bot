// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.Alignment;
import frc.robot.autos.TurretCalibrationCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.AllianceHandler;
import frc.robot.vision.PoseSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller joystick = new CommandPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private Alignment alignment = new Alignment();
    private final PoseSubsystem poseSubsystem = new PoseSubsystem(drivetrain);
    private TurretCalibrationCommand turretCalibrationCommand = new TurretCalibrationCommand(turret, poseSubsystem);

    public Translation2d blueHubPosition = new Translation2d(4.625, 4.04);
    public Translation2d redHubPosition = new Translation2d(11.925, 4.04);
    private boolean nearTrench = false;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());
        NamedCommands.registerCommand("RunIntakeCommand", Commands.startEnd(
            () -> intake.intakeDownAndIntakeCommand(() -> drivetrain.getFieldRelativeSpeed()),
            () -> intake.setIntakeVelocity(0)
        ));
        NamedCommands.registerCommand("IntakeOn", intake.beginIntakeCommand(() -> drivetrain.getFieldRelativeSpeed()));
        NamedCommands.registerCommand("IntakeOff", intake.endIntakeCommand());
        NamedCommands.registerCommand("IntakeDown", intake.intakeDown());
        NamedCommands.registerCommand("AgitateIntake", intake.intakeAgitate());
        //NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());
        NamedCommands.registerCommand("ShootAtHub", turret.getAutoAimAndShootCommand(poseSubsystem, drivetrain, () -> getHubPos(), nearTrench));

        NamedCommands.registerCommand("StopShooting", turret.runOnce(() -> {
            turret.stopMotors();
            turret.stopFeeding();
            turret.setHoodPosition(-0.3);
        }));


        
        //CHANGE AUTO NAME HERE.
        autoChooser = AutoBuilder.buildAutoChooser("T1ShootNeutral");
        SmartDashboard.putData("Auto Mode", autoChooser);
        //WILLIAM IF YOU FORGET YOU DIE A HORRIBLE DEATH. 

        configureBindings();
        
        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    public Translation2d getHubPos() {
        if (AllianceHandler.checkAllianceSide() == Alliance.Red) {
            return redHubPosition;
        }
        return blueHubPosition;
    }

    public double getXValueForDongle(){
        return (AllianceHandler.checkAllianceSide() == Alliance.Red) ? 12.0 : 4.5;
    }

    private void configureBindings() {
        double currentMaxSpeed;
        if (turret.isShooting) {
            currentMaxSpeed = MaxSpeed * 0.75;
        } else {
            currentMaxSpeed = MaxSpeed;
        }
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * currentMaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * currentMaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );



        turret.setDefaultCommand(
            turret.run(() -> {
                if (turret.aimingToggle) {
                    turret.autoAim(poseSubsystem.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), () -> getHubPos());
                    turret.setShooterVelocity(5.0);
                    if ((poseSubsystem.getCurrentPose().getX() >= 3.5 && poseSubsystem.getCurrentPose().getX() <= 5.75) //||
                         //poseSubsystem.getCurrentPose().getX() >= 13.5 && poseSubsystem.getCurrentPose().getX() <= 11.25
                    ) {
                        nearTrench = true;
                    } else {
                        nearTrench = false;
                    }
                } else {
                    turret.setHoodPosition(-0.3);
                    turret.setShooterVelocity(5.0);
                }
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.cross().onTrue(Commands.runOnce(() -> turret.toggleAiming()));

        joystick.L1().onTrue(
            Commands.either(
                alignment.toTrench1AS().andThen(alignment.toNeutralZoneFromT1()),
                alignment.toNZTrench1().andThen(alignment.toT1FromNZ()),
                AllianceHandler.checkAllianceSide() == Alliance.Red 
                ? () -> poseSubsystem.getCurrentPose().getX() > getXValueForDongle() 
                : () -> poseSubsystem.getCurrentPose().getX() < getXValueForDongle()
            )
        );

        joystick.R1().onTrue(
            Commands.either(
                alignment.toTrench2AS().andThen(alignment.toNeutralZoneFromT2()),
                alignment.toNZTrench2().andThen(alignment.toT2FromNZ()),
                AllianceHandler.checkAllianceSide() == Alliance.Red 
                ? () -> poseSubsystem.getCurrentPose().getX() > getXValueForDongle() 
                : () -> poseSubsystem.getCurrentPose().getX() < getXValueForDongle()
            )
        );
        
        joystick.L2().whileTrue(
            intake.intakeDownAndIntakeCommand(() -> drivetrain.getFieldRelativeSpeed())
        );

        joystick.R2().whileTrue(
            turret.getAutoAimAndShootCommand(poseSubsystem, drivetrain, () -> getHubPos(), nearTrench)
        );

        joystick.L2().and(joystick.L3().negate()).whileTrue(
            intake.intakeDownAndIntakeCommand(() -> drivetrain.getFieldRelativeSpeed())
        );

        joystick.L3().toggleOnTrue(intake.intakeAgitate());

        joystick.R3().toggleOnTrue(
            intake.intakeDownAndOuttakeCommand().alongWith(
            turret.outtakeHopper()));

        joystick.povUp().onTrue(intake.intakeUp());
        joystick.povDown().onTrue(intake.intakeDown());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
