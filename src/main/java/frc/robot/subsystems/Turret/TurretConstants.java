// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class TurretConstants {
    public static final TalonFX turretTurner = new TalonFX(50, "Aux");
    public static final TalonFX turretShooter = new TalonFX(51, "Aux");
    public static final TalonFX turretFeeder = new TalonFX(48, "Aux");
    public static final TalonFX turretHood = new TalonFX(44, "Aux");
    public static final TalonFX turretHopper = new TalonFX(49, "Aux");

    public static final TalonFX turretGuideL = new TalonFX(5, "Aux");
    public static final TalonFX turretGuideR = new TalonFX(6, "Aux");

    public static final CANcoder encoder11T = new CANcoder(1, "Aux"); 
    public static final CANcoder encoder10T = new CANcoder(2, "Aux"); 
}