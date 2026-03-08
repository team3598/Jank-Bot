package frc.robot.subsystems;

import frc.robot.utils.*;
import frc.robot.vision.PoseSubsystem;
import frc.robot.subsystems.*;
import frc.robot.autos.*;
import frc.robot.constants.TurretConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.LinearFilter; 
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

    public boolean aimingToggle = false;
    public boolean isShooting = false;

    public PoseSubsystem Pose;
    public CommandSwerveDrivetrain Drivetrain;
    public Translation2d targetHub = new Translation2d(4.625, 4.035);

    private final TalonFX turretTurner = TurretConstants.turretTurner;
    private final TalonFX turretShooter = TurretConstants.turretShooter;
    private final TalonFX turretFeeder = TurretConstants.turretFeeder;
    private final TalonFX turretHood = TurretConstants.turretHood;
    private final TalonFX turretHopper = TurretConstants.turretHopper;

    private final TalonFX turretGuideL = TurretConstants.turretGuideL;
    private final TalonFX turretGuideR = TurretConstants.turretGuideR;
    
    private final CANcoder enc10T = TurretConstants.encoder10T;
    private final CANcoder enc11T = TurretConstants.encoder11T;

    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final com.ctre.phoenix6.controls.VoltageOut zeroVolts = new com.ctre.phoenix6.controls.VoltageOut(0);
    private final com.ctre.phoenix6.controls.VoltageOut sysIdControl = new com.ctre.phoenix6.controls.VoltageOut(0);
    private final MotionMagicVoltage turnerMMRequest = new MotionMagicVoltage(0); 
    private final MotionMagicVoltage hoodMMRequest = new MotionMagicVoltage(0); 
    public InterpolatingDoubleTreeMap m_shooterSpeedMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap m_hoodAngleMap = new InterpolatingDoubleTreeMap();
    
    private final double shooterWheelRadius = Units.inchesToMeters(2.0); 
    
    private final double fuelEfficiency = 0.3; 
    private final double maxShift = 2;
    private final double rotationLookAhead = 0.1;
    private double currentAimTargetRotations = 0;

    private final SimpleCRT crtSolver = new SimpleCRT(10, 11, 100, -10.0, 360.0);
    
    public TurretSubsystem() {
        seedDistMaps();
        configureMotors();

        try { Thread.sleep(250); } catch (InterruptedException e) {}

        seedTurretPosition();

        turretHood.setPosition(0.0);
    }

    private final SysIdRoutine shooterSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            (voltage) -> turretShooter.setControl(sysIdControl.withOutput(voltage.in(Volts))),
            
            (log) -> {
                log.motor("ShooterFlywheel")
                   .voltage(Volts.of(turretShooter.getMotorVoltage().getValueAsDouble()))
                   .angularPosition(Rotations.of(turretShooter.getPosition().getValueAsDouble()))
                   .angularVelocity(RotationsPerSecond.of(turretShooter.getVelocity().getValueAsDouble()));
            },
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return shooterSysIdRoutine.dynamic(direction);
    }

    public void setDependencies(PoseSubsystem pose, CommandSwerveDrivetrain drivetrain) {
        this.Pose = pose;
        this.Drivetrain = drivetrain;
    }

    public boolean getAimingToggle() {
        return aimingToggle;
    }

    public void configureMotors() {
        final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Feedback.SensorToMechanismRatio = 1.6;
        flywheelConfig.Slot0.kP = 0.1;
        flywheelConfig.Slot0.kV = 0.3;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 60.0;

        final TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.Slot0.kP = 0;
        feederConfig.Slot0.kV = 0.1;
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        final TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.Slot0.kP = 0;
        hopperConfig.Slot0.kV = 0.1;
        hopperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.SupplyCurrentLimit = 30.0;

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Feedback.SensorToMechanismRatio = 125.0/3.0; 
        turnerConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0;
        turnerConfig.MotionMagic.MotionMagicAcceleration = 20.0;  
        turnerConfig.MotionMagic.MotionMagicJerk = 100.0;
        turnerConfig.Slot0.kP = 80; 
        turnerConfig.Slot0.kV = 0.35;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; 
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1.0;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turnerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        
        final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 12.0; 
        hoodConfig.MotionMagic.MotionMagicAcceleration = 24.0;
        hoodConfig.Slot0.kV = 0.1175;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 13;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5; 

        final TalonFXConfiguration guideLConfig = new TalonFXConfiguration();
        guideLConfig.Feedback.SensorToMechanismRatio = 3.0;
        guideLConfig.Slot0.kP = 0;
        guideLConfig.Slot0.kV = 0.1;
        guideLConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        guideLConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

        final TalonFXConfiguration guideRConfig = new TalonFXConfiguration();
        guideRConfig.Feedback.SensorToMechanismRatio = 3.0;
        guideRConfig.Slot0.kP = 0;
        guideRConfig.Slot0.kV = 0.1;
        guideRConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        guideRConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

        final CANcoderConfiguration enc10TConfiguration = new CANcoderConfiguration();
        enc10TConfiguration.MagnetSensor.MagnetOffset = 0.41;
        enc10TConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        enc10TConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        final CANcoderConfiguration enc11TConfiguration = new CANcoderConfiguration();
        enc11TConfiguration.MagnetSensor.MagnetOffset = 0.69;
        enc11TConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        enc11TConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        turretTurner.getConfigurator().apply(turnerConfig);
        turretShooter.getConfigurator().apply(flywheelConfig);
        turretFeeder.getConfigurator().apply(feederConfig);
        turretHood.getConfigurator().apply(hoodConfig);
        turretHopper.getConfigurator().apply(hopperConfig);
        turretGuideL.getConfigurator().apply(guideLConfig);
        turretGuideR.getConfigurator().apply(guideRConfig);
        enc10T.getConfigurator().apply(enc10TConfiguration);
        enc11T.getConfigurator().apply(enc11TConfiguration);
    }

    private void seedTurretPosition() {
        double posA = enc10T.getAbsolutePosition().getValueAsDouble(); 
        double posB = enc11T.getAbsolutePosition().getValueAsDouble();

        System.out.println(String.format("RAW: %.4f | %.4f", posA, posB));

        double solvedAngle = crtSolver.getTrueAngle(posA, posB);

        if (Double.isNaN(solvedAngle)) {
            System.out.println("CRT FAILURE: Sensors disagree or are out of bounds!");
            turretTurner.setPosition(0.0); 
        } else {
            System.out.println("CRT SUCCESS -> Angle: " + String.format("%.2f", solvedAngle));
            
            turretTurner.setPosition(-solvedAngle / 360.0);
        }
    }

    public void seedDistMaps() {
        m_shooterSpeedMap.put(1.75, 11.75);
        m_hoodAngleMap.put(1.75, 0.0);
        m_shooterSpeedMap.put(2.0, 12.0);
        m_hoodAngleMap.put(2.0, 0.0);
        m_shooterSpeedMap.put(2.5, 12.0);
        m_hoodAngleMap.put(2.5, 0.0);
        m_shooterSpeedMap.put(3.0, 12.0);
        m_hoodAngleMap.put(3.0, 4.0);
        m_shooterSpeedMap.put(3.5, 12.75);
        m_hoodAngleMap.put(3.5, 8.0);
        m_shooterSpeedMap.put(4.0, 13.25);
        m_hoodAngleMap.put(4.0, 8.5);
        m_shooterSpeedMap.put(4.5, 13.8);
        m_hoodAngleMap.put(4.5, 9.0);
        m_shooterSpeedMap.put(5.0, 14.25);
        m_hoodAngleMap.put(5.0, 10.0);
        m_shooterSpeedMap.put(5.5, 15.0);
        m_hoodAngleMap.put(5.5, 11.0);
    }

    public double autoAim(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Supplier<Translation2d> targetPosition) {
        double distance = robotPose.getTranslation().getDistance(targetPosition.get());
        double t = calculateTimeOfFlight(distance);
        double dragFactor = 0.85; 
        
        double targetX = targetPosition.get().getX();
        double targetY = targetPosition.get().getY();
        double virtX = targetX;
        double virtY = targetY;

        for (int i = 0; i < 4; i++) {
            double effectiveVx = fieldRelativeSpeeds.vxMetersPerSecond * dragFactor;
            double effectiveVy = fieldRelativeSpeeds.vyMetersPerSecond * dragFactor;

            double shiftX = -(effectiveVx * t);
            double shiftY = -(effectiveVy * t);
            
            shiftX = Math.max(-maxShift, Math.min(maxShift, shiftX));
            shiftY = Math.max(-maxShift, Math.min(maxShift, shiftY));

            virtX = targetX + shiftX;
            virtY = targetY + shiftY;
            
            double dx = virtX - robotPose.getX();
            double dy = virtY - robotPose.getY();
            distance = Math.sqrt((dx * dx) + (dy * dy));
            
            t = calculateTimeOfFlight(distance);
            if (t > 2.0) t = 2.0; 
        }

        double dx = virtX - robotPose.getX();
        double dy = virtY - robotPose.getY();
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double relativeAngle = targetAngleDeg - robotPose.getRotation().getDegrees();

        double aimAngle = Math.IEEEremainder(relativeAngle, 360.0);

        double robotSpinRPS = fieldRelativeSpeeds.omegaRadiansPerSecond / (2 * Math.PI);
        double finalTarget = aimAngle + (-robotSpinRPS * rotationLookAhead * 360.0);

        finalTarget = MathUtil.inputModulus(finalTarget, -360, 0);

        if (finalTarget > 0) finalTarget = 0;
        if (finalTarget < -360) finalTarget = -360;

        moveTurretAngle(finalTarget / 360.0);
        
        return distance;
    }

    public double calculateTimeOfFlight(double distance) {
        double targetRPS = m_shooterSpeedMap.get(distance);
        double targetHoodDeg = m_hoodAngleMap.get(distance); 

        double flywheelSurfaceSpeed = targetRPS * (2 * Math.PI * shooterWheelRadius);
        double exitVelocity = flywheelSurfaceSpeed * fuelEfficiency;

        double horizontalVelocity = exitVelocity * Math.cos(Math.toRadians(targetHoodDeg));

        if (horizontalVelocity < 1.0) return 1.0; 
        return (distance / horizontalVelocity) + 0.1;
    }

    public boolean isShooterAtSpeed(double targetRPS) {
        return Math.abs(turretShooter.getVelocity().getValueAsDouble() - targetRPS) < 3.5;
    }

    public boolean isHoodAtAngle(double targetAngle) {
        return Math.abs(turretHood.getPosition().getValueAsDouble() - targetAngle) < 1;
    }

    public boolean isTurretAligned(double toleranceDegrees){
        double currentAimedPosition = turretTurner.getPosition().getValueAsDouble();
        double error = Math.abs(currentAimedPosition - currentAimTargetRotations) * 360;
        return error < toleranceDegrees;
    }

    public void pulseHopper(double fastRPS, double slowRPS) {
        if (Timer.getFPGATimestamp() % 0.5 < 0.25) {
            setHopperVelocity(fastRPS);
        } else {
            setHopperVelocity(slowRPS);
        }
    }

    public void toggleAiming(){
        aimingToggle = !aimingToggle;
    }

    public Command outtakeHopper() {
        return runEnd(
            () -> setHopperVelocity(-100), 
            () -> setHopperVelocity(0));
    }

    public Command getAutoAimAndShootCommand(PoseSubsystem pose, CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> targetHub, boolean nearTrench) {
        return this.runEnd(
            () -> {
                double virtualDist = autoAim(pose.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), targetHub);
            
                double targetSpeed = m_shooterSpeedMap.get(virtualDist);
                double targetAngle = m_hoodAngleMap.get(virtualDist);
                setShooterVelocity(targetSpeed);
                
                if (!nearTrench){
                    setHoodPosition(targetAngle);
                } else {
                    setHoodPosition(-0.2);
                }
                
                if (isShooterAtSpeed(targetSpeed) && isHoodAtAngle(targetAngle)) {
                    setFeederVelocity(80);
                    setHopperVelocity(50);
                    setGuideVelocities(40);
                } else {
                    stopFeeding();    
                }
            },
            () -> {
                stopMotors();
                setShooterVelocity(5.0);
                stopFeeding();
                setHoodPosition(-0.3);
            }
        );
    }

    public Command getAutoAimAndShootCommandCalibration(PoseSubsystem pose, CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> targetHub, boolean nearTrench, DoubleSupplier hoodPosition, DoubleSupplier flywheelPower) {
        return this.runEnd(
            () -> {
                double virtualDist = autoAim(pose.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), targetHub);
            
                double targetSpeed = m_shooterSpeedMap.get(virtualDist);
                setShooterVelocity(flywheelPower.getAsDouble());

                if (!nearTrench){
                    setHoodPosition(hoodPosition.getAsDouble());
                } else {
                    setHoodPosition(0);
                }

                if (isShooterAtSpeed(flywheelPower.getAsDouble()) && isTurretAligned(4)) {
                    setFeederVelocity(90);
                    setHopperVelocity(50);
                    setGuideVelocities(50);
                } else {
                    stopFeeding(); 
                }
            },
            () -> {
                stopMotors();
                turretShooter.setControl(zeroVolts);
                stopFeeding();
                setHoodPosition(-0.3);
            }
        );
    }

    public void moveTurretAngle(double turretRotations) {
        this.currentAimTargetRotations = turretRotations;
        turretTurner.setControl(turnerMMRequest.withPosition(turretRotations));
    }
    
    public void stopMotors() {
        turretTurner.setControl(zeroVolts);
        turretFeeder.setControl(zeroVolts);
        turretHopper.setControl(zeroVolts);
        turretGuideL.setControl(zeroVolts);
        turretGuideR.setControl(zeroVolts);
    }

    public void stopFeeding() {
        turretFeeder.setControl(zeroVolts);
        turretHopper.setControl(zeroVolts);
        turretGuideL.setControl(zeroVolts);
        turretGuideR.setControl(zeroVolts);
    }

    public void setShooterVelocity(double targetRPS) {
        turretShooter.setControl(velocity.withVelocity(targetRPS));
    }

    public void setFeederVelocity(double rps) {
        turretFeeder.setControl(velocity.withVelocity(rps));
    }

    public void setHoodPosition(double position) {
        turretHood.setControl(hoodMMRequest.withPosition(position));
    }

    public void setHopperVelocity(double rps) {
        turretHopper.setControl(velocity.withVelocity(rps));
    }

    public void setGuideVelocities(double rps) {
        turretGuideL.setControl(velocity.withVelocity(-rps));
        turretGuideR.setControl(velocity.withVelocity(rps));
    }

    public double getFeederSpeed(){
        return turretFeeder.getVelocity().getValueAsDouble();
    }

    public double getFlywheelSpeed(){
        return turretShooter.getVelocity().getValueAsDouble();
    }

    public double getHopperSpeed()
    {
        return turretHopper.getVelocity().getValueAsDouble();
    }

    public double getGuideLSpeed()
    {
        return turretGuideL.getVelocity().getValueAsDouble();
    }

    
    @Override
    public void periodic() {
        //67
        }
    }