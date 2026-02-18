package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_intake = new TalonFX(46, "Aux");

    private final TalonFX m_intakeVL = new TalonFX(45, "Aux"); //VL stands for verticality left
    private final TalonFX m_intakeVR = new TalonFX(47, "Aux"); //VR stands for verticality right, this is the follower motor of VL

    private final Follower m_followRequest = new Follower(45, MotorAlignmentValue.Opposed);

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage intakeVerticalMotionMagic = new MotionMagicVoltage(0);

    public IntakeSubsystem() {
        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.12;
        configs.Slot0.kV = 0.12;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final TalonFXConfiguration intakeVConfig = new TalonFXConfiguration();
        intakeVConfig.Feedback.SensorToMechanismRatio = 1.0; 
        intakeVConfig.MotionMagic.MotionMagicCruiseVelocity = 35.0;
        intakeVConfig.MotionMagic.MotionMagicAcceleration = 100.0;  
        intakeVConfig.MotionMagic.MotionMagicJerk = 800.0;         
        intakeVConfig.Slot0.kP = 4.0; 
        intakeVConfig.Slot0.kI = 0.0;
        intakeVConfig.Slot0.kD = 0.15;
        intakeVConfig.Slot0.kG = 0.0; 
        intakeVConfig.Slot0.kV = 0.15;
        intakeVConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeVConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        intakeVConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        intakeVConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.05; 
        intakeVConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        intakeVConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -7.85; 
        intakeVConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var intake_status = m_intake.getConfigurator().apply(configs);
        var intakeV_status = m_intakeVL.getConfigurator().apply(intakeVConfig);
        var intakeVR_status = m_intakeVR.getConfigurator().apply(intakeVConfig);

        if (!intake_status.isOK()) {
            System.out.println("Could not configure Intake Motor: " + intake_status.toString());
        }

         if (!intakeV_status.isOK()) {
            System.out.println("Could not configure Intake Verticality Motors: " + intakeV_status.toString());
        }

        if (!intakeVR_status.isOK()) {
            System.out.println("Could not configure Intake Motor: " + intakeVR_status.toString());
        }

        m_intakeVL.setPosition(0);
    }

    public void setIntakeVelocity(double rps) {
        m_intake.setControl(m_velocity.withVelocity(rps));
        m_intakeVR.setControl(m_followRequest);
    }

    public void setIntakeVerticalityVelocity(double rps) {
        m_intakeVL.setControl(m_velocity.withVelocity(rps));
        m_intakeVR.setControl(m_followRequest);
        //insert intake verticality code here
    }
    
    public void setIntakeVerticalityPosition(double position) {
        m_intakeVL.setControl(intakeVerticalMotionMagic.withPosition(position));
        m_intakeVR.setControl(m_followRequest);
    }

    public Command moveIntakeUpOrDown(double rps) {
        return this.runEnd(
            () -> this.setIntakeVerticalityVelocity(rps),
            () -> this.m_intakeVL.stopMotor()
        );
    }

    public Command setIntakeVerticalPosition(double position) {
        return this.runEnd(
            () -> this.setIntakeVerticalityPosition(position),
            () -> this.m_intakeVL.stopMotor()
        );
    }

    public Command runIntakeCommand(double rps) {
        return this.runEnd(
        () -> this.setIntakeVelocity(30), 
        () -> this.m_intake.stopMotor());
    }

    public Command beginIntakeCommand() {
        return new InstantCommand(() -> this.setIntakeVelocity(30));
    }

    public Command endIntakeCommand() {
        return new InstantCommand(() -> this.m_intake.stopMotor());
    }

    public double getIntakeVelocity() {
        return m_intake.getVelocity().getValueAsDouble();
    } 

    @Override
    public void periodic() {
        m_intakeVR.setControl(m_followRequest);
        //System.out.println(m_intakeVL.getPosition());
        SmartDashboard.putNumber("IntakeVelocity", getIntakeVelocity());
    }
}

//-0.0576
//7.95