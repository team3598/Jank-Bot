// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretTurner = TurretConstants.turretTurner;
    private final TalonFX turretShooter = TurretConstants.turretShooter;
    private final TalonFX turretFeeder = TurretConstants.turretFeeder;
    private final TalonFX turretHood = TurretConstants.turretHood;
    private final TalonFX turretHopper = TurretConstants.turretHopper;
    private final CANcoder turretTurnerEncoder = TurretConstants.turretTurnerEncoder;
    //private final AnalogPotentiometer tTurnerPot = TurretConstants.turretTurnerPotentiometer;

    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage turnerMMRequest = new MotionMagicVoltage(0); 
    private final MotionMagicVoltage hoodMMRequest = new MotionMagicVoltage(0); 
    public InterpolatingDoubleTreeMap m_shooterSpeedMap;
    public InterpolatingDoubleTreeMap m_hoodAngleMap;
    public Translation2d hubPosition = new Translation2d(4.625, 4.035);
;
    private final double shooterWheelRadius = Units.inchesToMeters(2.0); // 4-inch wheel
    private final double fuelEfficiency = 1.10; 
    
    public TurretSubsystem() {
        m_shooterSpeedMap = new InterpolatingDoubleTreeMap();
        m_hoodAngleMap = new InterpolatingDoubleTreeMap();
        seedDistMaps();

        final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = 0.1;
        flywheelConfig.Slot0.kV = 0.14;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.Slot0.kP = 0;
        feederConfig.Slot0.kV = 0.1;
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        final TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.Slot0.kP = 0;
        hopperConfig.Slot0.kV = 0.1;
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final TalonFXConfiguration turnerConfig = new TalonFXConfiguration();
        turnerConfig.Feedback.SensorToMechanismRatio = 41.66666; //placeholder, change this
        turnerConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; 
        turnerConfig.MotionMagic.MotionMagicAcceleration = 2.0;  
        turnerConfig.MotionMagic.MotionMagicJerk = 10.0;         
        turnerConfig.Slot0.kP = 12; 
        turnerConfig.Slot0.kV = 0.1; 
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25; 
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turnerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.25;
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

        var turretTurnerStatus = turretTurner.getConfigurator().apply(turnerConfig);
        var turretShooterStatus = turretShooter.getConfigurator().apply(flywheelConfig);
        var turretFeederStatus = turretFeeder.getConfigurator().apply(feederConfig);
        var turretHoodStatus = turretHood.getConfigurator().apply(hoodConfig);
        var turretHopperStatus = turretHopper.getConfigurator().apply(hopperConfig);

        if (!turretTurnerStatus.isOK()) {
            System.out.println("Could not configure Turret Turner Motor: " + turretTurnerStatus.toString());
        }
        if (!turretShooterStatus.isOK()) {
            System.out.println("Could not configure Turret Shooter Motor: " + turretShooterStatus.toString());
        }
        if (!turretFeederStatus.isOK()) {
            System.out.println("Could not configure Turret Feeder Motor: " + turretFeederStatus.toString());
        }
        if (!turretHoodStatus.isOK()) {
            System.out.println("Could not configure Turret Hood Motor: " + turretHoodStatus.toString());
        }
        if (!turretHopperStatus.isOK()) {
            System.out.println("Could not configure Turret Hopper Motor: " + turretHopperStatus.toString());
        }

        //resetTurretAngle();
        turretTurner.setPosition(0.0);
        turretHood.setPosition(0.0); //the actual angle irl is 20, 20/360
    }

    public void seedDistMaps() {
        //m_shooterSpeedMap.put(1.5, 28.5); 
        //m_hoodAngleMap.put(1.5, 0.0);   

        m_shooterSpeedMap.put(2.0, 24.0);
        m_hoodAngleMap.put(2.0, 0.0);

        m_shooterSpeedMap.put(2.5, 27.5);
        m_hoodAngleMap.put(2.5, 0.0);

        m_shooterSpeedMap.put(3.0, 30.0);
        m_hoodAngleMap.put(3.0, 0.0);

        m_shooterSpeedMap.put(3.5, 31.0);
        m_hoodAngleMap.put(3.5, 2.5);

        m_shooterSpeedMap.put(4.0, 33.0);
        m_hoodAngleMap.put(4.0, 4.0);

        m_shooterSpeedMap.put(4.5, 34.0);
        m_hoodAngleMap.put(4.5, 6.0);

        m_shooterSpeedMap.put(5.0, 35.0);
        m_hoodAngleMap.put(5.0, 7.0);
    }

    /*public void resetTurretAngle() {
        double absolutePosition = turretTurnerEncoder.getAbsolutePosition().getValueAsDouble();
        double calibratedPosition = absolutePosition - TurretConstants.turretTurnerOffset;
        turretTurner.setPosition(calibratedPosition);
    }*/

    public double calculateTimeOfFlight(double distance) {
        double targetRPS = m_shooterSpeedMap.get(distance);
        double flywheelSurfaceSpeed = targetRPS * (2 * Math.PI * shooterWheelRadius);
    
        double exitVelocity = flywheelSurfaceSpeed * fuelEfficiency;

        if (exitVelocity < 1.0) return 1.0; 
        return (distance / exitVelocity) + 0.05;
    }

    public boolean isShooterAtSpeed(double targetRPS) {
        return Math.abs(turretShooter.getVelocity().getValueAsDouble() - targetRPS) < 1;
    }

    public void moveTurretAngle(double turretRotations) {
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

    public void setHoodAngle(double degrees) {
        turretHood.setControl(hoodMMRequest.withPosition(degrees/360));
    }

    public void setHopperSpeed(double rps) {
        turretHopper.setControl(velocity.withVelocity(rps));
    }
    
    public double getFeederSpeed(){
        return turretFeeder.getVelocity().getValueAsDouble();
    }

    public double getFlywheelSpeed(){
        return turretShooter.getVelocity().getValueAsDouble();
    }

    public Translation2d getCorrectedTargetPosition(Translation2d targetPos, ChassisSpeeds robotVel, PoseSubsystem poseSubsystem) {
        Pose2d robotPose = poseSubsystem.getCurrentPose();
        double distance = robotPose.getTranslation().getDistance(targetPos);

        double t = calculateTimeOfFlight(distance);

        double shiftX = -robotVel.vxMetersPerSecond * t;
        double shiftY = -robotVel.vyMetersPerSecond * t;

        return new Translation2d(
            targetPos.getX() + shiftX,
            targetPos.getY() + shiftY
        );
    }

    public Command goToHoodAngle(double degrees) {
       return this.runEnd(
        () -> {
                setHoodAngle(degrees);
                System.out.println("hood on. desired degrees of hood: " + degrees);
        },
        () -> turretHood.stopMotor()
        );
    }

    public Command aimAtTarget(Translation2d targetPosition, Supplier<Pose2d> currentPose) {
        return this.runEnd(
            () -> {
                double dx = targetPosition.getX() - currentPose.get().getX();
                double dy = targetPosition.getY() - currentPose.get().getY();

                Rotation2d angleToTarget = new Rotation2d(dx, dy);

                Rotation2d targetAngleRelative = angleToTarget.minus(currentPose.get().getRotation());
                moveTurretAngle(targetAngleRelative.getDegrees());
                //System.out.println(targetAngleRelative.getDegrees());
            },
            () -> moveTurretAngle(0)
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
                setHopperSpeed(50);
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
        //include a print statement for absolute encoder offset, then set that later.
        //System.out.println(turretShooter.getVelocity());
        //System.out.println(turretHood.getPosition());
        System.out.println("Turret Turn Position: " + turretTurner.getPosition());

    // This method will be called once per scheduler run
    }
}
