package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.REVUtils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.util.hardware.SparkMaxUtil;

public class IntakePivot extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private AbsoluteEncoder encoder;

    private SingleJointedArmSim armSim;
    private SparkMaxSim motorSim;

    private Timer deltaTimer;

    public IntakePivot() {
        motor = new SparkMax(CAN.INTAKE_PIVOT, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        motorConfig = new SparkMaxConfig();
        SparkMaxUtil.configure(motorConfig, false, IdleMode.kBrake);
        SparkMaxUtil.configurePID(motorConfig, 1.0, 0.0, 0.0, 0.0, false);
        SparkMaxUtil.saveAndLog(this, motor, motorConfig);

    closedLoopController = motor.getClosedLoopController();

        if (RobotBase.isSimulation()) {
            armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                Constants.INTAKE.PIVOT_GEARING,
                SingleJointedArmSim.estimateMOI(Inches.of(16.5).in(Meters), Pounds.of(4.455).in(Kilograms)),
                Inches.of(16.5).in(Meters),
                Units.degreesToRadians(0),
                Units.degreesToRadians(90.0),
                true,
                0.0
            );

            motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1).withReduction(Constants.INTAKE.PIVOT_GEARING));

            deltaTimer = new Timer();
        }

        setName("Intake/Pivot");
        setDefaultCommand(raise().repeatedly());

        Logger.logNumber(getName() + "/measuredAngleRotations", () -> encoder.getPosition());
    }

    private void setReference(Angle angle) {
        REVUtils.check(closedLoopController.setReference(angle.in(Rotations), ControlType.kPosition));
        // REVUtils.check(closedLoopController.setReference(angle.in(Rotations), ControlType.kPosition));
        
        Logger.log(getName() + "/targetAngleRotations", angle.in(Rotations));
    }

    public Command setAngle(Supplier<Angle> angle) {
        return Commands.run(
                () -> setReference(angle.get()), this)
                    .until(() -> motor.get() == angle.get().in(Rotations));
    }

    public Command lower() {
        return setAngle(() -> Preferences.INTAKE.PIVOT_DOWN);
    }

    public Command raise() {
        return setAngle(() -> Preferences.INTAKE.PIVOT_UP);
    }

    @Override
    public void periodic() {
        if (!ENABLED_SYSTEMS.INTAKE) {
            motor.disable();
        }
    }

    @Override
    public void simulationPeriodic() {
        deltaTimer.stop();

        Time delta = Seconds.of(deltaTimer.get());

        deltaTimer.reset();
        deltaTimer.start();

        // TODO: Decide if should be stored as a Measure or a double
        double motorOutputVoltage = motorSim.getAppliedOutput() * motorSim.getBusVoltage();

        Logger.log(getName() + "/simulatedMotorOutputVolts", motorOutputVoltage);
        armSim.setInputVoltage(motorOutputVoltage);
        armSim.update(delta.in(Seconds));

        Logger.log(getName() + "/armSpeedRadPerSec", armSim.getVelocityRadPerSec());

        motorSim.iterate(
            Radians.of(armSim.getVelocityRadPerSec()).in(Rotations),
            RobotController.getBatteryVoltage(),
            delta.in(Seconds)
        );
    }
}
