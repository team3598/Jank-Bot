package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// 2026 REV Imports
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intake1 = new TalonFX(46, "Aux");
    private final SparkFlex m_intake2 = new SparkFlex(44, MotorType.kBrushless);
    private final SparkClosedLoopController m_intake2PID = m_intake2.getClosedLoopController();

    private final TalonFX m_intakeVL = new TalonFX(45, "Aux");
    private final TalonFX m_intakeVR = new TalonFX(47, "Aux"); 

    private final Follower m_VFollowRequest = new Follower(45, MotorAlignmentValue.Opposed);

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage intakeVerticalMotionMagic = new MotionMagicVoltage(0);

    public IntakeSubsystem() {
        var talonFXconfigs = new TalonFXConfiguration();
        talonFXconfigs.Slot0.kP = 0.12;
        talonFXconfigs.Slot0.kV = 0.14;
        talonFXconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_intake1.getConfigurator().apply(talonFXconfigs);

        SparkFlexConfig vortexConfig = new SparkFlexConfig();
        
        vortexConfig.closedLoop.pidf(0.0001, 0.0, 0.0, 0.00017);
        vortexConfig.inverted(true);
        vortexConfig.idleMode(IdleMode.kBrake);
        vortexConfig.smartCurrentLimit(60);

        m_intake2.configure(vortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        final TalonFXConfiguration intakeVConfig = new TalonFXConfiguration();
        intakeVConfig.Feedback.SensorToMechanismRatio = 1.0; 
        intakeVConfig.MotionMagic.MotionMagicCruiseVelocity = 35.0;
        intakeVConfig.MotionMagic.MotionMagicAcceleration = 100.0;  
        intakeVConfig.MotionMagic.MotionMagicJerk = 800.0;         
        intakeVConfig.Slot0.kP = 4.0; 
        intakeVConfig.Slot0.kD = 0.15;
        intakeVConfig.Slot0.kV = 0.15;
        intakeVConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeVConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        intakeVConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        intakeVConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.14; 
        intakeVConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        intakeVConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.1;
        intakeVConfig.CurrentLimits.SupplyCurrentLimit = 30;
        intakeVConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeVConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_intakeVL.getConfigurator().apply(intakeVConfig);
        m_intakeVR.getConfigurator().apply(intakeVConfig);

        m_intakeVL.setPosition(0);
        m_intakeVR.setPosition(0);
    }

    public void setIntakeVelocity(double rps) {
        m_intake1.setControl(m_velocity.withVelocity(rps));
        // REV expects RPM
        m_intake2PID.setReference(rps * 30.0, ControlType.kVelocity);
        
        m_intakeVR.setControl(m_VFollowRequest);
    }

    public void setIntakeVerticalityVelocity(double rps) {
        m_intakeVL.setControl(m_velocity.withVelocity(rps));
        m_intakeVR.setControl(m_VFollowRequest);
    }
    
    public void setIntakeVerticalityPosition(double position) {
        m_intakeVL.setControl(intakeVerticalMotionMagic.withPosition(position));
        m_intakeVR.setControl(m_VFollowRequest);
    }

    public Command setIntakeVerticalPosition(double position) {
        return this.runEnd(
            () -> this.setIntakeVerticalityPosition(position),
            () -> this.m_intakeVL.stopMotor()
        );
    }

    public Command intakeUp() {
        return this.runOnce(() -> setIntakeVerticalityPosition(6));
    }
    
    public Command intakeDown() {
        return this.runOnce(() -> this.setIntakeVerticalityPosition(-0.05)
           );
    }

    public Command runIntakeCommand(double rps) {
        return this.runEnd(
            () -> this.setIntakeVelocity(30), 
            () -> {
                this.m_intake1.stopMotor();
                this.m_intake2.stopMotor(); 
            }
        );
    }

    public Command beginIntakeCommand() {
        return this.runOnce(() -> this.setIntakeVelocity(30));
    }

    public Command endIntakeCommand() {
        return this.runOnce(() -> {
            this.m_intake1.stopMotor();
            this.m_intake2.stopMotor(); 
        });
    }

    public double getIntakeVelocity() {
        return m_intake1.getVelocity().getValueAsDouble();
    } 

    @Override
    public void periodic() {
        m_intakeVR.setControl(m_VFollowRequest);
        SmartDashboard.putNumber("IntakeVelocity", getIntakeVelocity());
        //System.out.println(m_intakeVL.getPosition().getValueAsDouble());
    }
}