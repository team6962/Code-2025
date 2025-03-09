package frc.robot.subsystems.funnel;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.FUNNEL;
import frc.robot.Constants.RobotVersion;
import frc.robot.util.hardware.SparkMaxUtil;

public abstract class FunnelMotor extends SubsystemBase {
    public FunnelMotor() {
        setName("Funnel/Motor");
        setDefaultCommand(stop());

        Logger.logBoolean(getName() + "/running", this::isIntaking);
        Logger.logBoolean(getName() + "/disabled", this::isDisabled);
    }

    public boolean isDisabled() {
        return false;
    }

    protected abstract boolean isIntaking();
    public abstract Command stop();
    public abstract Command intake();
    public abstract Command fineControl(double power);

    public static FunnelMotor get() {
        if (!RobotVersion.isV2()) {
            throw new IllegalStateException("Funnel is not supported on version 1 robot");
        }

        return RobotBase.isReal() ? new Real() : new Simulated();
    }

    static class Real extends FunnelMotor {
        private SparkMax motor;
        private boolean intaking;

        public Real() {
            this.motor = new SparkMax(CAN.FUNNEL, MotorType.kBrushless);
            
            SparkMaxConfig config = new SparkMaxConfig();
            SparkMaxUtil.configure550(config, FUNNEL.MOTOR_INVERTED, IdleMode.kCoast);
            SparkMaxUtil.saveAndLog(this, motor, config);

            setDefaultCommand(stop());
        }

        @Override
        public Command stop() {
            return run(() -> {
                motor.set(0);
                intaking = false;
            });
        }

        @Override
        public Command intake() {
            return run(() -> {
                motor.set(FUNNEL.INTAKE_SPEED);
                intaking = true;
            });
        }

        @Override
        public Command fineControl(double power) {
            return run(() -> {
                motor.set(power * FUNNEL.INTAKE_SPEED);
                if (power > 0) intaking = true;
            });
        }

        @Override
        protected boolean isIntaking() {
            return intaking;
        }
    }

    static class Simulated extends FunnelMotor {
        private boolean intaking;

        public Simulated() {
            setDefaultCommand(stop());
        }

        @Override
        public Command stop() {
            return run(() -> intaking = false);
        }

        @Override
        public Command intake() {
            return run(() -> intaking = true);
        }

        @Override
        public Command fineControl(double power) {
            return run(() -> {
                if (power > 0) intaking = true;
            });
        }

        @Override
        protected boolean isIntaking() {
            return intaking;
        }
    }
}
