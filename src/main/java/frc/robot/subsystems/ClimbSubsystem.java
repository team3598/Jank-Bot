// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.*;

import java.io.IOException;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.AllianceHandler;
import frc.robot.vision.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.*;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  public static final TalonFX climbR = new TalonFX(99, "Aux"); //old id was 3
  private final VelocityVoltage velocity = new VelocityVoltage(0);
  private final PositionVoltage position = new PositionVoltage(0);
  private final com.ctre.phoenix6.controls.VoltageOut zeroVolts = new com.ctre.phoenix6.controls.VoltageOut(0);
  private final Pose2d blueTowerLeftSide = new Pose2d(1.375, 3.44, Rotation2d.fromDegrees(180));
  public CommandSwerveDrivetrain Drivetrain;

  public ClimbSubsystem() {
    final TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        //climbConfig.Feedback.SensorToMechanismRatio = 1.6;
        climbConfig.Slot0.kV = 0.4;
        climbConfig.Slot0.kP = 0.5;
        climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        climbConfig.CurrentLimits.StatorCurrentLimit = 100.0;

        climbR.getConfigurator().apply(climbConfig);
        climbR.setPosition(0);
        
  }

  public void setDependencies(CommandSwerveDrivetrain drivetrain) {
        this.Drivetrain = drivetrain;
    }

  public Command doDaFunnyClimb() {
    
    setClimbPosition(206);

    return Commands.defer(() -> {
      

        if (Drivetrain == null) {
            return Commands.print("ERROR: Drivetrain dependency not set!");
        }

        try {
            PathPlannerPath climbTrajectory = PathPlannerPath.fromChoreoTrajectory("ClimbTheTowerOfHell");

            PathConstraints constraints = new PathConstraints(
                4, 4, 
                Units.degreesToRadians(540), 
                Units.degreesToRadians(720)
            );

            return Commands.sequence(
                AutoBuilder.pathfindToPose(
                    climbTrajectory.getStartingHolonomicPose().get(), 
                    constraints, 
                    0.0 
                ),
                AutoBuilder.followPath(climbTrajectory),
                Commands.runEnd(
                () -> setClimbPosition(106), 
                () -> setClimbPosition(0)
                )
            );
        
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.print("FATAL: Could not load Choreo trajectory 'ClimbTheTowerOfHell'!");
        }

    }, Set.of((Subsystem) Drivetrain)); 
  }
  
  public void setClimbPosition(double pos) {
    climbR.setControl(position.withPosition(pos));
  }

  public void setClimbSpeed(double rps) {
    climbR.setControl(velocity.withVelocity(rps));
  }

  @Override
  public void periodic() {
    //System.out.println(climbR.getPosition());
    // This method will be called once per scheduler run
  }
}

