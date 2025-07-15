package com.team6962.lib.swerve;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team6962.lib.swerve.module.SwerveModule;

import edu.wpi.first.units.measure.LinearVelocity;

public interface SwerveMovement {
    public static PositionVoltage positionVoltage = new PositionVoltage(0);
    public static VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    public static MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
    public static MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    public static DynamicMotionMagicVoltage dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0, 0);
    
    public void execute(SwerveCore drivetrain);
    public SwerveMovement cleared();
}
