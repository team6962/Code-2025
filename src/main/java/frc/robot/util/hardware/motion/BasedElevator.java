package frc.robot.util.hardware.motion;

import com.badlogic.gdx.graphics.g3d.particles.ParticleSorter.Distance;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasedElevator extends SubsystemBase {
    public static class Config {
        public double kP;
        public double kI;
        public double kD;
        public double kS;
        public double kV;
        public double kA;
        public double kG;

        public LinearVelocity maxVelocity;
        public LinearAcceleration maxAcceleration;

        public Distance minHeight;
        public Distance maxHeight;

        public double gearReduction;
        public Distance sprocketRadius;

        public MotorConfig[] motors;

        public Current freeCurrentLimit;
        public Current stallCurrentLimit;

        public Distance tolerance;
    }

    public static class MotorConfig {
        public String name;
        public int canId;
        public boolean inverted;
    }

    public static class LimitSwitchConfig {
        public int dio;
        public LimitSwitchWiring wiring;
    }

    public static enum LimitSwitchWiring {
        NormallyOpen,
        NormallyClosed
    }
}
