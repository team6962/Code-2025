package frc.robot.util.hardware.motion;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator extends BasedElevator {
    private ElevatorSim elevatorSim;
    private SparkMaxSim[] motorSims;
    private Timer timer = new Timer();

    public SimElevator(BasedElevator.Config config) {
        super(config);

        DCMotor combinedMotors = new DCMotor(
            config.simulation.motor.nominalVoltageVolts,
            config.simulation.motor.stallTorqueNewtonMeters,
            config.simulation.motor.stallCurrentAmps,
            config.simulation.motor.freeCurrentAmps,
            config.simulation.motor.freeSpeedRadPerSec,
            config.motors.length
        );

        elevatorSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                combinedMotors,
                config.simulation.mass.in(Kilograms),
                config.sprocketRadius.in(Meters),
                config.gearReduction
            ),
            combinedMotors,
            config.minHeight.in(Meters),
            config.maxHeight.in(Meters),
            true,
            config.minHeight.in(Meters)
        );

        motorSims = new SparkMaxSim[config.motors.length];

        for (int i = 0; i < config.motors.length; i++) {
            motorSims[i] = motors[i].createSparkMaxSim();
        }
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        if (!timer.isRunning()) {
            timer.start();
            return;
        }

        double dt = timer.get();
        double appliedVoltage = 0.0;

        for (SparkMaxSim motorSim : motorSims) {
            appliedVoltage += motorSim.getAppliedOutput() * motorSim.getBusVoltage();
        }

        elevatorSim.setInputVoltage(appliedVoltage);
        elevatorSim.update(dt);

        for (SparkMaxSim motorSim : motorSims) {
            motorSim.setPosition(elevatorSim.getPositionMeters());
            motorSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());
            motorSim.setMotorCurrent(elevatorSim.getCurrentDrawAmps());
            motorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), 12.0, dt);
        }

        timer.restart();

        floor.setSimulatedState(elevatorSim.getPositionMeters() < config.minHeight.in(Meters) + 0.001);
        ceiling.setSimulatedState(elevatorSim.getPositionMeters() > config.maxHeight.in(Meters) - 0.001);
    }
}
