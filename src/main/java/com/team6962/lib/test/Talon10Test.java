package com.team6962.lib.test;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.utils.CTREUtils;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Talon10Test extends SubsystemBase {
    private TalonFX talon;

    public Talon10Test() {
        talon = new TalonFX(52);

        talon.getConfigurator().apply(new FeedbackConfigs()
            .withRotorToSensorRatio(1)
            /* .withSensorToMechanismRatio(1) */);
    }

    @Override
    public void periodic() {
        if (RobotState.isEnabled()) {
            CTREUtils.check(talon.getSimState().setRawRotorPosition(Degrees.of(100)));

            System.out.println(CTREUtils.unwrap(talon.getPosition()
                .waitForUpdate(1000)).in(Degrees));

            talon.close();
        }
    }
}
