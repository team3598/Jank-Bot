// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.AutoRoutines;
import frc.robot.autos.TurretCalibrationCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.AllianceHandler;
import frc.robot.vision.VisionSubsystem;

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

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final VisionSubsystem poseSubsystem = new VisionSubsystem(drivetrain);
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    public Translation2d blueHubPosition = new Translation2d(4.625, 4.04);
    public Translation2d redHubPosition = new Translation2d(11.925, 4.04);
    public Translation2d blue1PassPosition = new Translation2d(2.000, 6.700); //heh 67 heheheh
    public Translation2d blue2PassPosition = new Translation2d(2.000, 1.500);
    public Translation2d red1PassPosition = new Translation2d(14.000, 1.500); 
    public Translation2d red2PassPosition = new Translation2d(14.000, 6.700);
    private TurretCalibrationCommand turretCalibrationCommand = new TurretCalibrationCommand(turret, poseSubsystem);
    private boolean nearTrench = false;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        turret.setDependencies(poseSubsystem, drivetrain);
        climb.setDependencies(drivetrain);
        turretCalibrationCommand.ignoringDisable(true).schedule();

        autoFactory = drivetrain.createAutoFactory();
        autoFactory
        //.bind("DeployIntake", intake.intakeDown())
        .bind("Intaking", intake.intakeOutAndIntakeCommand(() -> drivetrain.getFieldRelativeSpeed()))
        .bind("IntakeOff", intake.endIntakeCommand())
        .bind("AgitateIntake", intake.intakeAgitate())
        .bind("BeginOutake", intake.beginOutakeCommand())
        .bind("ShootAtHub", turret.getAutoAimAndShootCommand(poseSubsystem, drivetrain, () -> getHubPos(), () -> nearTrench))
        .bind("StopShooting", turret.runOnce(() -> {
            turret.isShooting = false;
            turret.stopMotors();
            turret.setShooterVelocity(5.0);
            turret.stopFeeding();
            turret.setHoodPosition(-0.3);
        }));

        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser
        .addRoutine("T1OPRoboticsAuto", autoRoutines::T1OPRoboticsAuto)
        .addRoutine("T2OPRoboticsAuto", autoRoutines::T2OPRoboticsAuto)
        .addRoutine("ShootingTest", autoRoutines::shootingTest)
        .addRoutine("T2BeachBoys", autoRoutines::T2BeachBoys);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

     public Translation2d getHubPos() {
        if (AllianceHandler.checkAllianceSide() == Alliance.Red) {
            return redHubPosition;
        }
        return blueHubPosition;
    }

    public Translation2d getPassPos() {
        boolean isRed = (AllianceHandler.checkAllianceSide() == Alliance.Red);
        boolean isUpperHalf = (poseSubsystem.getCurrentPose().getY() > 4.025);

        if (isRed) { 
            return isUpperHalf ? red2PassPosition : red1PassPosition;
        } else {
            return isUpperHalf ? blue1PassPosition : blue2PassPosition;
        }
    }
    

    public double getXValueForDongle(){
        return (AllianceHandler.checkAllianceSide() == Alliance.Red) ? 12.0 : 4.5;
    }

    private void configureBindings() {
        Trigger isStarving = new Trigger(() -> turret.getFlywheelStatorCurrent() < 50.0).and(() -> turret.isShooting)
        .debounce(0.5);

        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
            boolean needSlow = turret.isShooting || intake.isIntaking;
            double joystickX = needSlow ? MathUtil.clamp(-joystick.getLeftY(), -0.2, 0.2) : -joystick.getLeftY();
            double joystickY = needSlow ? MathUtil.clamp(-joystick.getLeftX(), -0.2, 0.2) : -joystick.getLeftX();

            /*//double speedMultiplier = needSlow ? 0.4 : 1.0; 
            double filteredX = xLimiter.calculate(joystickX);
            double filteredY = yLimiter.calculate(joystickY);
            double filteredRot = rotLimiter.calculate(-joystick.getRightX());*/


            return drive.withVelocityX(joystickX * MaxSpeed)// * speedMultiplier)
                    .withVelocityY(joystickY * MaxSpeed)// * speedMultiplier)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
            })
        );

        turret.setDefaultCommand(
            turret.run(() -> {
                if (turret.aimingToggle) {
                    turret.autoAim(poseSubsystem.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), () -> getHubPos());
                    turret.setShooterVelocity(5.0); //5.0
                    //turret.setSpindexerVelocity(15);
                    if ((poseSubsystem.getCurrentPose().getX() >= 3.5 && poseSubsystem.getCurrentPose().getX() <= 5.75) //||
                         //poseSubsystem.getCurrentPose().getX() >= 13.5 && poseSubsystem.getCurrentPose().getX() <= 11.25
                    ) {
                        nearTrench = true;
                    } else {
                        nearTrench = false;
                    }
                } else {
                    turret.setHoodPosition(-0.3);
                    turret.stopMotors();
                    turret.setShooterVelocity(5.0); //5.0
                    //turret.setSpindexerVelocity(15);
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
        
        /* 
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
        */

        joystick.R2().whileTrue(  
            turret.getAutoAimAndShootCommandCalibration(poseSubsystem, drivetrain, () -> getHubPos(), nearTrench, () -> turretCalibrationCommand.hoodTunerNumber, () -> turretCalibrationCommand.flywheelTunerNumber)
            .alongWith(
            intake.slowlyAgitateAndSpinCommand()
            )
        ); //this is for calibration and tuning
        
        /*joystick.R2().whileTrue(
            turret.getAutoAimAndShoot(poseSubsystem, drivetrain, () -> getHubPos(), () -> nearTrench)
            //.alongWith(
            //intake.intakeAgitate())
            //.alongWith(
            //intake.slowlyAgitateAndSpinCommand()
            //)
        );*/ //actual shooting

        joystick.R1().whileTrue(
            turret.getAutoAimAndShoot(poseSubsystem, drivetrain, () -> getPassPos(), () -> nearTrench)
        );

        joystick.L3().and(isStarving).onTrue(
            intake.slowlyAgitateAndSpinCommand()
        );
        

        joystick.L2().and(joystick.L3().negate()).whileTrue(
                intake.intakeOutAndIntakeCommand(() -> drivetrain.getFieldRelativeSpeed())
        );

        //joystick.square().toggleOnTrue(intake.intakeAgitate());

        /*joystick.R3().whileTrue(
            intake.intakeDownAndOuttakeCommand()
        );*/

        joystick.povUp().whileTrue(
            Commands.runEnd(
                () -> intake.applyIntakeRackVelocity(3), 
                () -> intake.applyIntakeRackVelocity(0))
            );

        joystick.povDown().whileTrue(
            Commands.runEnd(
                () -> intake.applyIntakeRackVelocity(-3),
                () -> intake.applyIntakeRackVelocity(0))
        );

        joystick.povLeft().onTrue(intake.intakeClosed());

        joystick.povRight().onTrue(intake.intakeOpen());

        /*joystick.povLeft().whileTrue(Commands.runEnd(
            () -> climb.setClimbSpeed(30),
            () -> climb.setClimbSpeed(0)
        ));

        joystick.povRight().whileTrue(Commands.runEnd(
            () -> climb.setClimbSpeed(-30),
            () -> climb.setClimbSpeed(0)
        ));
        */

        /*joystick.circle().onTrue(
            climb.doDaFunnyClimb()
        );
        */

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
