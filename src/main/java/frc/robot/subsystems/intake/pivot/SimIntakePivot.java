package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.INTAKE;
import java.util.function.Supplier;

public class SimIntakePivot extends IntakePivot {
  private SingleJointedArmSim armSim;
  private PIDController pidController;
  private Voltage outputVoltage = Volts.of(0);

  public SimIntakePivot() {
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            INTAKE.GEARING,
            SingleJointedArmSim.estimateMOI(INTAKE.LENGTH.in(Meters), INTAKE.MASS.in(Kilograms)),
            INTAKE.LENGTH.in(Meters),
            Degrees.of(0).in(Radians),
            Degrees.of(90).in(Radians),
            false, // Pretending that we have feedforward in sim
            Degrees.of(90).in(Radians));

    pidController = new PIDController(INTAKE.PID.kP, INTAKE.PID.kI, INTAKE.PID.kD);
    pidController.setSetpoint(0.25);
    pidController.setTolerance(Degrees.of(1.0).in(Radians));
  }

  private Angle getAngle() {
    return Radians.of(armSim.getAngleRads());
  }

  @Override
  public Command setAngle(Supplier<Angle> angle) {
    return Commands.run(
            () -> {
              outputVoltage =
                  Volts.of(
                      pidController.calculate(getAngle().in(Rotations), angle.get().in(Rotations)));
            },
            this)
        .until(() -> pidController.atSetpoint());
  }

  @Override
  public void periodic() {
    armSim.setInputVoltage(outputVoltage.in(Volts));
    armSim.update(0.02);
  }

  @Override
  protected Voltage getAppliedVoltage() {
    return outputVoltage;
  }

  @Override
  protected Angle getMeasuredAngle() {
    return getAngle();
  }

  @Override
  protected Angle getTargetAngle() {
    return Rotations.of(pidController.getSetpoint());
  }
}
