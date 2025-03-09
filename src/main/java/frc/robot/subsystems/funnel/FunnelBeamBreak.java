package frc.robot.subsystems.funnel;

import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.telemetry.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.RobotVersion;

public abstract class FunnelBeamBreak extends SubsystemBase {
    public FunnelBeamBreak() {
        setName("Funnel/Beam Break");

        Logger.logBoolean(getName() + "/detected", this::detectsGamePiece);
    }

    public abstract boolean detectsGamePiece();

    public void expectGamePiece() {
    }

    public static FunnelBeamBreak get(FunnelMotor motor) {
        if (!RobotVersion.isV2()) {
            throw new IllegalStateException("Funnel is not supported on version 1 robot");
        }

        if (ENABLED_SYSTEMS.FUNNEL) {
            return RobotBase.isReal() ? new Real() : new Simulated((FunnelMotor.SimulatedOrDisabled) motor);
        } else {
            return new Disabled((FunnelMotor.SimulatedOrDisabled) motor);
        }
    }

    static class Real extends FunnelBeamBreak {
        private DigitalInput input;
        private Debouncer debouncer;
        private boolean detected;

        public Real() {
            this.input = new DigitalInput(DIO.FUNNEL_BEAM_BREAK);
            this.debouncer = new Debouncer(0.05);
        }

        @Override
        public boolean detectsGamePiece() {
            return detected;
        }

        @Override
        public void periodic() {
            detected = debouncer.calculate(!input.get());
        }
    }

    static class Simulated extends FunnelBeamBreak {
        private FunnelMotor.SimulatedOrDisabled motor;
        private Time dropStart;
        private boolean detected = false;

        private static final Time DROP_TIME = Seconds.of(0.25);

        public Simulated(FunnelMotor.SimulatedOrDisabled motor) {
            this.motor = motor;
        }

        @Override
        public boolean detectsGamePiece() {
            return detected;
        }

        @Override
        public void expectGamePiece() {
            detected = true;
        }

        @Override
        public void periodic() {
            Time now = Seconds.of(Timer.getFPGATimestamp());

            if (motor.isIntaking() && dropStart == null && detected) {
                dropStart = now;
            }

            if (dropStart != null) {
                Time timeAfterDropStart = now.minus(dropStart);

                if (timeAfterDropStart.gt(DROP_TIME)) {
                    detected = false;
                    dropStart = null;
                }
            }
        }
    }

    static class Disabled extends FunnelBeamBreak {
        private FunnelMotor.SimulatedOrDisabled motor;
        private boolean detected = false;

        public Disabled(FunnelMotor.SimulatedOrDisabled motor) {
            this.motor = motor;
        }

        @Override
        public boolean detectsGamePiece() {
            return detected;
        }

        @Override
        public void expectGamePiece() {
            detected = true;
        }

        @Override
        public void periodic() {
            if (motor.isIntaking()) detected = false;
        }
    }
}
