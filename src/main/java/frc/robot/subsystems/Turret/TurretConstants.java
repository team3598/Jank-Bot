// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

/** Add your docs here. */
public class TurretConstants {
    //motor declarations
    public static final TalonFX turretTurner = new TalonFX(50, "Aux");
    public static final TalonFX turretShooter = new TalonFX(51, "Aux");
    public static final TalonFX turretFeeder = new TalonFX(48, "Aux");
    public static final TalonFX turretHood = new TalonFX(52, "Aux");
    public static final TalonFX turretHopper = new TalonFX(49, "Aux");
    public static final AnalogPotentiometer turretTurnerPotentiometer = new AnalogPotentiometer(4);
    public static final double pOffset = 0.000339498078;

    //temporary
    public static final CANcoder turretTurnerEncoder = new CANcoder(6, "Aux");
    public static final double turretTurnerOffset = 0.5; 
}
