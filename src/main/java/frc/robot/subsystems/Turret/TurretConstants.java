// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class TurretConstants {
    public static final TalonFX turretTurner = new TalonFX(0, "Drive Base");
    public static final TalonFX turretShooter = new TalonFX(5,   "Drive Base");
    //public static final TalonFX turretFeeder = new TalonFX(3, "Drive Base");
    public static final SparkFlex turretFeeder = new SparkFlex(41, MotorType.kBrushless);
    public static final TalonFX turretHood = new TalonFX(1, "Drive Base");
    public static final TalonFX turretHopper = new TalonFX(2, "Drive Base");
    public static final TalonFX turretWillyL = new TalonFX(4, "Drive Base");
    public static final TalonFX turretWillyR = new TalonFX(6, "Drive Base");

//43 
    
    
    //public static final CANcoder encoder11T = new CANcoder(1, "Drive Base"); 
    //public static final CANcoder encoder10T = new CANcoder(2, "Drive Base"); 
}