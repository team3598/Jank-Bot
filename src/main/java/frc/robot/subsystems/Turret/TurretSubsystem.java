// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import frc.robot.SimpleCRT;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.commands.Autos.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretTurner = TurretConstants.turretTurner;
    private final TalonFX turretShooter = TurretConstants.turretShooter;
    private final TalonFX turretFeeder = TurretConstants.turretFeeder;
    private final TalonFX turretHood = TurretConstants.turretHood;
    private final TalonFX turretHopper = TurretConstants.turretHopper;
    
    //private final CANcoder enc10T = TurretConstants.encoder10T;
    //private final CANcoder enc11T = TurretConstants.encoder11T;

    private static final int TEETH_A = 10;
    private static final int TEETH_B = 11;
    private static final int TURRET_RING_TEETH = 100;

    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage turnerMMRequest = new MotionMagicVoltage(0); 
    private final MotionMagicVoltage hoodMMRequest = new MotionMagicVoltage(0); 
    public InterpolatingDoubleTreeMap m_shooterSpeedMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap m_hoodAngleMap = new InterpolatingDoubleTreeMap();
    
    private final double shooterWheelRadius = Units.inchesToMeters(2.0); 
    
    private final double fuelEfficiency = 0.35; 
    private final double maxShift = 2; 
    private final double rotationLookAhead = 0.1;
    private double currentAimTargetRotations = 0;

    private double jamStartTime = -1.0;
    private double unjamEndTime = -1.0;

    private final SimpleCRT crtSolver = new SimpleCRT(10, 11, 100, -10.0, 360.0);
    
    public TurretSubsystem() {
        seedDistMaps();
        configureMotors();

        try { Thread.sleep(250); } catch (InterruptedException e) {}

        //seedTurretPosition();

        turretTurner.setPosition(0.0); 
        turretHood.setPosition(0.0);
    }

    public void configureMotors() {
        final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = 0.1;
        flywheelConfig.Slot0.kV = 0.15;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.Slot0.kP = 0;
        feederConfig.Slot0.kV = 0.1;
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        final TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.Slot0.kP = 0;
        hopperConfig.Slot0.kV = 0.1;
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //hopperConfig.CurrentLimits.StatorCurrentLimit = 60.0;

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Feedback.SensorToMechanismRatio = 41.66666; 
        turnerConfig.MotionMagic.MotionMagicCruiseVelocity = 8.0; 
        turnerConfig.MotionMagic.MotionMagicAcceleration = 20.0;  
        turnerConfig.MotionMagic.MotionMagicJerk = 100.0;
        turnerConfig.Slot0.kP = 80; 
        turnerConfig.Slot0.kV = 0.35;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.4; 
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.4;
        turnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Feedback.SensorToMechanismRatio = 250.0; 
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 0.75; 
        hoodConfig.MotionMagic.MotionMagicAcceleration = 1.0;  
        hoodConfig.MotionMagic.MotionMagicJerk = 10.0;         
        hoodConfig.Slot0.kP = 60; 
        hoodConfig.Slot0.kD = 0.1; 
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.3; 

        turretTurner.getConfigurator().apply(turnerConfig);
        turretShooter.getConfigurator().apply(flywheelConfig);
        turretFeeder.getConfigurator().apply(feederConfig);
        turretHood.getConfigurator().apply(hoodConfig);
        turretHopper.getConfigurator().apply(hopperConfig);
    }

    /*private void seedTurretPosition() {
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
    }*/

    public void seedDistMaps() {
        m_shooterSpeedMap.put(2.0, 24.0);
        m_hoodAngleMap.put(2.0, 0.0);

        m_shooterSpeedMap.put(2.5, 26.5);
        m_hoodAngleMap.put(2.5, 0.0);

        m_shooterSpeedMap.put(3.0, 28.0);
        m_hoodAngleMap.put(3.0, 0.0);

        m_shooterSpeedMap.put(3.5, 29.0);
        m_hoodAngleMap.put(3.5, 2.5);

        m_shooterSpeedMap.put(4.0, 31.0);
        m_hoodAngleMap.put(4.0, 3.5);

        m_shooterSpeedMap.put(4.5, 32.0);
        m_hoodAngleMap.put(4.5, 5.0);

        m_shooterSpeedMap.put(5.0, 33.0);
        m_hoodAngleMap.put(5.0, 5.5);
    }

    public double autoAim(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetPosition) {
        double realDist = robotPose.getTranslation().getDistance(targetPosition);
        double t = calculateTimeOfFlight(realDist);
        if (t > 1.0) t = 1.0;
        double shiftX = -fieldRelativeSpeeds.vxMetersPerSecond * t;
        double shiftY = -fieldRelativeSpeeds.vyMetersPerSecond * t;
        shiftX = Math.max(-maxShift, Math.min(maxShift, shiftX));
        shiftY = Math.max(-maxShift, Math.min(maxShift, shiftY));
        Translation2d virtualHub = new Translation2d(targetPosition.getX() + shiftX, targetPosition.getY() + shiftY);
        Translation2d robotToTarget = targetPosition.minus(robotPose.getTranslation());
        Rotation2d angleToTarget = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
        double velTowardsTarget = (fieldRelativeSpeeds.vxMetersPerSecond * angleToTarget.getCos()) + (fieldRelativeSpeeds.vyMetersPerSecond * angleToTarget.getSin());
        double virtualDist = realDist - (velTowardsTarget * t);

        double dx = virtualHub.getX() - robotPose.getX();
        double dy = virtualHub.getY() - robotPose.getY();
        Rotation2d targetRot = new Rotation2d(dx, dy);
        Rotation2d relativeRot = targetRot.minus(robotPose.getRotation());
        double aimAngle = Math.IEEEremainder(relativeRot.getDegrees(), 360.0);
        //System.out.println(aimAngle);

        double robotSpinRPS = fieldRelativeSpeeds.omegaRadiansPerSecond / (2 * Math.PI);
        
        double finalTarget = -(aimAngle + (-robotSpinRPS * rotationLookAhead * 360.0));
        //finalTarget = MathUtil.inputModulus(finalTarget, -360, 0);

        moveTurretAngle(((finalTarget / 360.0)));
        
        return virtualDist;
    }

    public double calculateTimeOfFlight(double distance) {
        double targetRPS = m_shooterSpeedMap.get(distance);
        double targetHoodDeg = m_hoodAngleMap.get(distance); 

        double flywheelSurfaceSpeed = targetRPS * (2 * Math.PI * shooterWheelRadius);
        double exitVelocity = flywheelSurfaceSpeed * fuelEfficiency;

        double horizontalVelocity = exitVelocity * Math.cos(Math.toRadians(targetHoodDeg));

        if (horizontalVelocity < 1.0) return 1.0; 
        return (distance / horizontalVelocity) + 0.05;
    }   

    public boolean isShooterAtSpeed(double targetRPS) {
        return Math.abs(turretShooter.getVelocity().getValueAsDouble() - targetRPS) < 1.5;
    }

    public boolean isTurretAligned(double toleranceDegrees){
        double currentAimedPosition = turretTurner.getPosition().getValueAsDouble();
        double error = Math.abs(currentAimedPosition - currentAimTargetRotations) * 360;
        return error < toleranceDegrees;
    }

    public boolean runFeederWithUnjam() {
        double currentTime = PoseSubsystem.timeMS;

        if (unjamEndTime > 0 && currentTime < unjamEndTime) {
            setHopperVelocity(-40);
            setFeederVelocity(-20);
            jamStartTime = -1.0; 
            return true; 
        }

        unjamEndTime = -1.0; 

        double currentAmps = turretHopper.getStatorCurrent().getValueAsDouble();
    
        if (currentAmps >= 50.0) {
            if (jamStartTime < 0) {
                jamStartTime = currentTime; 
            }
            if (currentTime - jamStartTime > 100.0) {
                unjamEndTime = currentTime + 500.0; 
                jamStartTime = -1.0; 
            } else {
                setFeederVelocity(90);
                setHopperVelocity(50);
            }
        } else {
            jamStartTime = -1.0; 
            setFeederVelocity(90);
            setHopperVelocity(50);
        }
        return false;
    }

    public void stopFeeding() {
        unjamEndTime = -1.0;
        jamStartTime = -1.0;
        setFeederVelocity(0);
        setHopperVelocity(0);
    }

    public Command getAutoAimAndShootCommand(PoseSubsystem pose, CommandSwerveDrivetrain drivetrain, Translation2d targetHub, boolean nearTrench) {
        return this.runEnd(
            () -> {
                double virtualDist = autoAim(pose.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), targetHub);
            
                double targetSpeed = m_shooterSpeedMap.get(virtualDist);
                setShooterVelocity(targetSpeed);

                if (!nearTrench){
                    setHoodPosition(m_hoodAngleMap.get(virtualDist));
                } else {
                    setHoodPosition(0);
                }

                if (isShooterAtSpeed(targetSpeed) && isTurretAligned(1.5)) {
                    runFeederWithUnjam();
                } else {
                    stopFeeding(); 
                }
            },
            () -> {
                stopMotors();
                stopFeeding();
                setHoodPosition(-0.3);
            }
        );
    }

    public Command getAutoAimAndShootCommandCalibration(PoseSubsystem pose, CommandSwerveDrivetrain drivetrain, Translation2d targetHub, boolean nearTrench, DoubleSupplier hoodPosition, DoubleSupplier flywheelPower) {
        return this.runEnd(
            () -> {
                double virtualDist = autoAim(pose.getCurrentPose(), drivetrain.getFieldRelativeSpeed(), targetHub);
            
                double targetSpeed = m_shooterSpeedMap.get(virtualDist);
                //setShooterVelocity(targetSpeed);
                setShooterVelocity(flywheelPower.getAsDouble());

                if (!nearTrench){
                    //setHoodPosition(m_hoodAngleMap.get(virtualDist));
                    setHoodPosition(hoodPosition.getAsDouble());
                } else {
                    setHoodPosition(0);
                }

                if (isShooterAtSpeed(flywheelPower.getAsDouble()) && isTurretAligned(1.5)) {
                    runFeederWithUnjam();
                } else {
                    stopFeeding(); 
                }
            },
            () -> {
                stopMotors();
                stopFeeding();
                setHoodPosition(-0.3);
            }
        );
    }
    public void moveTurretAngle(double turretRotations) {
        this.currentAimTargetRotations = turretRotations;
        turretTurner.setControl(turnerMMRequest.withPosition(turretRotations));
    }
    
    public void stopMotors(){
        turretTurner.stopMotor();
        turretHood.stopMotor();
        turretShooter.stopMotor();
        turretFeeder.stopMotor();
        turretHopper.stopMotor();
    }

    public void setShooterVelocity(double rps) {
        turretShooter.setControl(velocity.withVelocity(rps));
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

    public Command goToHoodAngle(double degrees) {
       return this.runEnd(
        () -> {
                setHoodPosition(degrees);
                //System.out.println("hood on. desired degrees of hood: " + degrees);
        },
        () -> turretHood.stopMotor()
        );
    }

    public Command shootTurret(double rps) {
        return this.runOnce(
            () -> {
                setShooterVelocity(rps);
            }
        ).andThen(
            waitUntil(()->isShooterAtSpeed(rps))
        ).andThen(
            this.run(()->{
                setFeederVelocity(100);
                setHopperVelocity(50);
            })
        ).finallyDo(
            (interrupted)->{
            turretShooter.stopMotor();
            turretFeeder.stopMotor();
            turretHopper.stopMotor();
            }
        );
    }

    public Command goToAngle(double degrees) {
        return this.runEnd(
            () -> moveTurretAngle(degrees / 360.0),
            () -> turretTurner.stopMotor()
        );
    } 

    @Override
    public void periodic() {
        //System.out.println("FlywheelSpeed: " + getFlywheelSpeed() + ", Turret Turner Position: " + turretTurner.getPosition().getValueAsDouble());
        //double error = turretTurner.getClosedLoopError().getValueAsDouble();
        //System.out.println("Tracking Error: " + error);
        //System.out.println("Turret Degrees: " + turretTurner.getPosition().getValueAsDouble() * 360);
        System.out.println(turretHood.getPosition().getValueAsDouble());
    }
}