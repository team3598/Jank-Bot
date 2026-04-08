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

    private final TalonFX m_intakeL = new TalonFX(29, "Aux");
    private final TalonFX m_intakeR = new TalonFX(28, "Aux");

    private final TalonFX m_intakeRL = new TalonFX(6, "Aux"); //rack left
    private final TalonFX m_intakeRR = new TalonFX(7, "Aux"); //rack right

    private final Follower m_RFollowRequest = new Follower(6, MotorAlignmentValue.Opposed);
    private final Follower m_intakeFollowRequest = new Follower(29, MotorAlignmentValue.Opposed);


    private final VelocityVoltage m_velocity = new VelocityVoltage(0);
    private final MotionMagicVoltage intakeRackMotionMagic = new MotionMagicVoltage(0).withSlot(0);
    private final DynamicMotionMagicVoltage slowIntakeRackMotionMagic = new DynamicMotionMagicVoltage(0, 4, 5.0).withSlot(0);
    
    public IntakeSubsystem() {
        final VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakForwardVoltage = 12.0; 
        voltageConfigs.PeakReverseVoltage = -12.0; 
        
        var talonFXconfigs = new TalonFXConfiguration();
        talonFXconfigs.Slot0.kP = 0.12;
        talonFXconfigs.Slot0.kV = 0.14;
        talonFXconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXconfigs.CurrentLimits.StatorCurrentLimit = 60;
        talonFXconfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXconfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXconfigs.Feedback.SensorToMechanismRatio = 1.0;
        m_intakeL.getConfigurator().apply(talonFXconfigs);

        final TalonFXConfiguration intakeRConfig = new TalonFXConfiguration();
        intakeRConfig.Feedback.SensorToMechanismRatio = 1.0;
        intakeRConfig.MotionMagic.MotionMagicCruiseVelocity = 12.0;
        intakeRConfig.MotionMagic.MotionMagicAcceleration = 36.0;  
        //intakeRConfig.MotionMagic.MotionMagicJerk = 800.0;   
        intakeRConfig.Slot0.kP = 0.3; 
        intakeRConfig.Slot0.kD = 0.0;
        intakeRConfig.Slot0.kV = 0.3;
        intakeRConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeRConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        intakeRConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 33.75; 
        intakeRConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        intakeRConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;
        intakeRConfig.CurrentLimits.StatorCurrentLimit = 30;
        intakeRConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeRConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_intakeL.getConfigurator().apply(voltageConfigs);
        m_intakeR.getConfigurator().apply(voltageConfigs);

        m_intakeRL.getConfigurator().apply(intakeRConfig);
        m_intakeRR.getConfigurator().apply(intakeRConfig);
        m_intakeRL.getConfigurator().apply(voltageConfigs); 
        m_intakeRR.getConfigurator().apply(voltageConfigs);
        m_intakeRL.setPosition(0);
        m_intakeRR.setPosition(0);

        m_intakeRR.setControl(m_RFollowRequest);
    }

    public void setIntakeVelocity(double rps) {
        m_intakeL.setControl(m_velocity.withVelocity(rps));
        m_intakeR.setControl(m_intakeFollowRequest);
        //m_intakeRR.setControl(m_VFollowRequest);
    }

    public void applyIntakeRackVelocity(double rps) {
        m_intakeRL.setControl(m_velocity.withVelocity(rps));
        m_intakeRR.setControl(m_RFollowRequest);
    }

    public double calculatedIntakeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        double robotVelocity = Math.sqrt((fieldRelativeSpeeds.vxMetersPerSecond * fieldRelativeSpeeds.vxMetersPerSecond) + (fieldRelativeSpeeds.vyMetersPerSecond * fieldRelativeSpeeds.vyMetersPerSecond));
        double robotVelocityInFeet = robotVelocity * 3.281;
        double targetIntakeVelocity = robotVelocityInFeet * 2;
        double intakeSpinSpeed = Math.max(50, targetIntakeVelocity * 2 * Math.PI);

        return intakeSpinSpeed;
    }

    private void applyIntakeVelocity(double rps) {
        m_intakeL.setControl(m_velocity.withVelocity(rps));
        m_intakeR.setControl(m_intakeFollowRequest);
    }
    
    private void zeroIntakeVerticalVoltage() {
        this.m_intakeRL.setVoltage(0);
    }
    
    private void applyIntakePosition(double position) {
        m_intakeRL.setControl(intakeRackMotionMagic.withPosition(position));
        //m_intakeRR.setControl(m_RFollowRequest);
    }

    public Command setIntakeRackPosition(double position) {
        return this.runEnd(
            () -> this.setIntakeRackPosition(position),
            () -> zeroIntakeVerticalVoltage()
        );
    }
    
    public Command intakeOpen() {
        return this.runOnce(() -> this.applyIntakePosition(32.5));
    }
    
    public Command intakeClosed() {
        return this.run(() -> {
            this.applyIntakePosition(-0.05);
            this.setIntakeVelocity(30);
        })
        .until(() -> Math.abs(m_intakeRL.getPosition().getValueAsDouble() - (-0.05)) < 0.1)
        .finallyDo((interrupted) -> {
            this.setIntakeVelocity(0);
            }
        );
    }

    public Command intakePositionZeroVolts() {
        return this.runOnce(() -> zeroIntakeVerticalVoltage());
    }

    public Command intakeAgitate(){
        return this.runEnd(
            () -> {
                if (Timer.getFPGATimestamp() % 1.0 < 1.1) {
                    this.applyIntakePosition(-0.05);
                    this.setIntakeVelocity(10);
                } else {
                    this.applyIntakePosition(3.75);
                    this.setIntakeVelocity(10);
                }
            },
            
            () -> {
                this.setIntakeVelocity(0);
                this.applyIntakePosition(-0.05);
            });
    }

    /*public Command intakePop() {
        return this.runEnd(
            ,
            );
    }*/

    public Command intakeOutAndIntakeCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return this.runEnd(() -> {
            //applyIntakePosition(32.5);
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

            //m_intakeRL.setControl(intakeRackMotionMagic.withPosition(currentPos));
            applyIntakePosition(currentPos);
            applyIntakeVelocity(10);
        })
        .beforeStarting(timer::restart)
        .finallyDo(() -> {
            setIntakeVelocity(0);
            applyIntakePosition(0.1); 
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
            this.setIntakeRackPosition(32.5);
            });
    }

    public Command endIntakeCommand() {
        return this.runOnce(() -> {
            this.m_intakeL.stopMotor();
            this.m_intakeR.setControl(m_intakeFollowRequest);
            zeroIntakeVerticalVoltage();
        });
    }

     
    public Command beginOutakeCommand() {
        return this.run(() -> {
            this.setIntakeVelocity(-40);
            //this.setIntakeRackPosition(-0.1);
            });
    }



    public Command endOutakeCommand() {
        return this.runOnce(() -> {
            this.m_intakeL.stopMotor();
            this.m_intakeR.setControl(m_intakeFollowRequest);
            zeroIntakeVerticalVoltage();
        });
    }
    public double getIntakeVelocity() {
        return m_intakeL.getVelocity().getValueAsDouble();
    } 


    @Override
    public void periodic() {
        //ChassisSpeeds speeds = drivetrain.getFieldRelativeSpeed();
        //m_intakeRR.setControl(m_VFollowRequest);
        //SmartDashboard.putNumber("IntakeVelocity", getIntakeVelocity());
        System.out.println(m_intakeRL.getPosition().getValueAsDouble());
    }
}