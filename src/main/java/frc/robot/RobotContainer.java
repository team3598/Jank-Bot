package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem.TurretState; 
import frc.robot.commands.Autos.Alignment;
import frc.robot.commands.TurretCalibrationCommand;
import frc.robot.LimelightHelpers;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SendableChooser<Command> autoChooser;

    private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0);
    private final PS4Controller HIDController = new PS4Controller(1);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final PoseSubsystem poseSubsystem = new PoseSubsystem(drivetrain);
    
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private Alignment alignment = new Alignment();
    private TurretCalibrationCommand turretCalibrationCommand = new TurretCalibrationCommand(turret, poseSubsystem);
    
    public Translation2d hubPosition = new Translation2d(4.625, 4.035);

    public RobotContainer() {
        turret.setDependencies(poseSubsystem, drivetrain);

        configureBindings();
        
        NamedCommands.registerCommand("ShootAtHub", turret.getAutoAimAndShootCommand(hubPosition));
        NamedCommands.registerCommand("StopShooting", Commands.runOnce(() -> turret.setState(TurretState.IDLE)));
    
        NamedCommands.registerCommand("IntakeOn", Commands.runOnce(() -> intake.setState(IntakeStates.DOWN_INTAKING)));
        NamedCommands.registerCommand("IntakeOff", Commands.runOnce(() -> intake.setState(IntakeStates.IDLE)));
        NamedCommands.registerCommand("IntakeUp", Commands.runOnce(() -> intake.setState(IntakeStates.FOLDED)));
        NamedCommands.registerCommand("IntakeDown", Commands.runOnce(() -> intake.setState(IntakeStates.DOWN)));
        
        NamedCommands.registerCommand("RunIntakeCommand", Commands.startEnd(
            () -> intake.setState(IntakeStates.DOWN_INTAKING),
            () -> intake.setState(IntakeStates.IDLE)
        ));

        NamedCommands.registerCommand("AlignToTower", alignment.alignToTower());

        autoChooser = AutoBuilder.buildAutoChooser("StatesTest");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        turretCalibrationCommand.ignoringDisable(true).schedule();
        FollowPathCommand.warmupCommand().schedule();
    }
    
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double currentMaxSpeed = (turret.getState() == TurretState.SHOOTING) ? (MaxSpeed / 1.5) : MaxSpeed;
                return drive.withVelocityX(-ps5Controller.getLeftY() * currentMaxSpeed)
                            .withVelocityY(-ps5Controller.getLeftX() * currentMaxSpeed) 
                            .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        ps5Controller.cross().onTrue(
            Commands.runOnce(() -> turret.toggleAiming())
        );

        ps5Controller.R2().whileTrue(
            Commands.runEnd(
                () -> {
                    turret.setState(TurretState.SHOOTING);
                    intake.setState(IntakeStates.DOWN_INTAKING);
                    HIDController.setRumble(RumbleType.kBothRumble, 1);
                    HIDController.setOutput(1, true);
                },
                () -> {
                    turret.setState(turret.getAimingToggle() ? TurretState.AIMING : TurretState.IDLE);
                    intake.setState(IntakeStates.IDLE);
                    HIDController.setRumble(RumbleType.kBothRumble, 0);
                    HIDController.setOutput(0, false);
                }
            )
        );

        ps5Controller.L2().toggleOnTrue(
            Commands.runEnd(
                () -> {
                    turret.setState(TurretState.INTAKING_HOPPER);
                    intake.setState(IntakeStates.DOWN_INTAKING);
                },
                () -> {
                    turret.setState(turret.getAimingToggle() ? TurretState.AIMING : TurretState.IDLE);
                    intake.setState(IntakeStates.IDLE);
                }
            )
        );

        ps5Controller.R3().toggleOnTrue(
            Commands.runEnd(
                () -> {
                    turret.setState(TurretState.REVERSING_HOPPER);
                    intake.setState(IntakeStates.OUTTAKING);
                },
                () -> {
                    turret.setState(turret.getAimingToggle() ? TurretState.AIMING : TurretState.IDLE);
                    intake.setState(IntakeStates.IDLE);
                }
            )
        );

        ps5Controller.L3().whileTrue(
            Commands.startEnd(
                () -> intake.setState(IntakeStates.AGITATING),
                () -> {
                    if (turret.getState() == TurretState.SHOOTING) {
                        intake.setState(IntakeStates.DOWN_INTAKING);
                    } else {
                        intake.setState(IntakeStates.IDLE);
                    }
                }
            )
        );

        ps5Controller.povUp().onTrue(intake.setIntakeVerticalPosition(6.14));
        ps5Controller.povDown().onTrue(intake.setIntakeVerticalPosition(-0.05));
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}