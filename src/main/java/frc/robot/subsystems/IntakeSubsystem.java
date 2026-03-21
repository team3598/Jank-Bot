package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TunerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase {

    public boolean isAgitating = false;
    public boolean isIntaking = false;

    private final TalonFX m_intake1 = new TalonFX(46, "Aux");

    private final TalonFX m_intakeVL = new TalonFX(45, "Aux");
    private final TalonFX m_intakeVR = new TalonFX(47, "Aux"); 

    private final Follower m_VFollowRequest = new Follower(45, MotorAlignmentValue.Opposed);

    private final VelocityVoltage m_velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage intakeVerticalMotionMagic = new MotionMagicVoltage(0).withSlot(0);
    private final DynamicMotionMagicVoltage slowIntakeVerticalMotionMagic = new DynamicMotionMagicVoltage(0, 4, 5.0).withSlot(0);
    
    public IntakeSubsystem() {
        final VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakForwardVoltage = 12.0; 
        voltageConfigs.PeakReverseVoltage = -12.0; 
        
        var talonFXconfigs = new TalonFXConfiguration();
        talonFXconfigs.Slot0.kP = 0.12;
        talonFXconfigs.Slot0.kV = 0.14;
        talonFXconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXconfigs.CurrentLimits.StatorCurrentLimit = 80;
        talonFXconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXconfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXconfigs.Feedback.SensorToMechanismRatio = 3.0;
        m_intake1.getConfigurator().apply(talonFXconfigs);

        final TalonFXConfiguration intakeVConfig = new TalonFXConfiguration();
        intakeVConfig.Feedback.SensorToMechanismRatio = 1.0; 
        intakeVConfig.MotionMagic.MotionMagicCruiseVelocity = 35.0;
        intakeVConfig.MotionMagic.MotionMagicAcceleration = 100.0;  
        intakeVConfig.MotionMagic.MotionMagicJerk = 800.0;         
        intakeVConfig.Slot0.kP = 4.0; 
        intakeVConfig.Slot0.kD = 0.15;
        intakeVConfig.Slot0.kV = 0.15;

        intakeVConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        intakeVConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.14; 
        intakeVConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        intakeVConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.3;
        intakeVConfig.CurrentLimits.StatorCurrentLimit = 35;
        intakeVConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeVConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_intake1.getConfigurator().apply(voltageConfigs);

        m_intakeVL.getConfigurator().apply(intakeVConfig);
        m_intakeVR.getConfigurator().apply(intakeVConfig);

        m_intakeVL.setPosition(0);
        m_intakeVR.setPosition(0);

        m_intakeVR.setControl(m_VFollowRequest);
    }

    public void setIntakeVelocity(double rps) {
        m_intake1.setControl(m_velocity.withVelocity(rps));
        
        //m_intakeVR.setControl(m_VFollowRequest);
    }

    public void setIntakeVerticalityVelocity(double rps) {
        m_intakeVL.setControl(m_velocity.withVelocity(rps));
        //m_intakeVR.setControl(m_VFollowRequest);
    }
    
    public void setIntakeVerticalityPosition(double position) {
        m_intakeVL.setControl(intakeVerticalMotionMagic.withPosition(position));
        //m_intakeVR.setControl(m_VFollowRequest);
    }

    public double calculatedIntakeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        double robotVelocity = Math.sqrt((fieldRelativeSpeeds.vxMetersPerSecond * fieldRelativeSpeeds.vxMetersPerSecond) + (fieldRelativeSpeeds.vyMetersPerSecond * fieldRelativeSpeeds.vyMetersPerSecond));
        double robotVelocityInFeet = robotVelocity * 3.281;
        double targetIntakeVelocity = robotVelocityInFeet * 2;
        double intakeSpinSpeed = Math.max(50, targetIntakeVelocity * 2 * Math.PI);

        return intakeSpinSpeed;
    }

    private void applyIntakeVelocity(double rps) {
        m_intake1.setControl(m_velocity.withVelocity(rps));
        //m_intakeVR.setControl(m_VFollowRequest);
    }
    
    private void zeroIntakeVerticalVoltage() {
        this.m_intakeVL.setVoltage(0);
    }
    
    private void applyVerticalPosition(double position) {
        m_intakeVL.setControl(intakeVerticalMotionMagic.withPosition(position));
        //m_intakeVR.setControl(m_VFollowRequest);
    }

    public Command setIntakeVerticalPosition(double position) {
        return this.runEnd(
            () -> this.setIntakeVerticalityPosition(position),
            () -> zeroIntakeVerticalVoltage()
        );
    }
    
    public Command intakeUp() {
        return this.runOnce(() -> this.setIntakeVerticalityPosition(6.14));
    }
    
    public Command intakeDown() {
        return this.runOnce(() -> this.setIntakeVerticalityPosition(-0.1)
           );
    }   

    public Command intakeVerticalZeroVolts() {
        return this.runOnce(() -> zeroIntakeVerticalVoltage());
    }

    public Command intakeAgitate(){
        return this.runEnd(
            () -> {
                if (Timer.getFPGATimestamp() % 1.0 < 0.3) {
                    this.setIntakeVerticalityPosition(3);
                    this.setIntakeVelocity(10);
                } else {
                    this.setIntakeVerticalityPosition(0.5);
                    this.setIntakeVelocity(10);
                }
            },
            
            () -> {
                this.setIntakeVelocity(0);
                this.setIntakeVerticalityPosition(-0.05);
            });
    }

    /*public Command intakePop() {
        return this.runEnd(
            ,
            );
    }*/

    public Command intakeDownAndIntakeCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return this.runEnd(() -> {
            applyVerticalPosition(-0.1);
            double targetSpeed = calculatedIntakeSpeeds(chassisSpeeds.get());
            applyIntakeVelocity(targetSpeed);  
            isIntaking = true;
        }, 
            () -> {
                setIntakeVelocity(0);
                zeroIntakeVerticalVoltage();
                isIntaking = false;
            }
        );
    }

    public Command slowlyAgitateAndSpinCommand() {  
        Timer timer = new Timer();

        return this.run(() -> {
            int intervals = (int) (timer.get() / 0.75);
        
            double currentPos = 1.5 + (0.75 * (intervals % 3)); 

            //m_intakeVL.setControl(intakeVerticalMotionMagic.withPosition(currentPos));
            applyVerticalPosition(currentPos);
            applyIntakeVelocity(10);
        })
        .beforeStarting(timer::restart)
        .finallyDo(() -> {
            setIntakeVelocity(0);
            applyVerticalPosition(-0.1); 
        });
    }

    public Command intakeDownAndOuttakeCommand() {
        return this.runEnd(
            () -> {
                applyIntakeVelocity(-50);
                zeroIntakeVerticalVoltage();
            },
            () -> {
                applyIntakeVelocity(0);
                zeroIntakeVerticalVoltage();
            });
    }
    
    public Command beginIntakeCommand(Supplier<ChassisSpeeds> chassisSpeed) {
        return this.run(() -> {
            this.setIntakeVelocity(calculatedIntakeSpeeds(chassisSpeed.get()));
            this.setIntakeVerticalityPosition(-0.1);
            });
    }

    public Command endIntakeCommand() {
        return this.runOnce(() -> {
            this.m_intake1.stopMotor();
            zeroIntakeVerticalVoltage();
        });
    }

    public double getIntakeVelocity() {
        return m_intake1.getVelocity().getValueAsDouble();
    } 


    @Override
    public void periodic() {
        //ChassisSpeeds speeds = drivetrain.getFieldRelativeSpeed();
        //m_intakeVR.setControl(m_VFollowRequest);
        //SmartDashboard.putNumber("IntakeVelocity", getIntakeVelocity());
        //System.out.println(m_intakeVL.getPosition().getValueAsDouble());
    }
}