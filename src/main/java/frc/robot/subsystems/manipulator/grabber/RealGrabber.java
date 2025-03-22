package frc.robot.subsystems.manipulator.grabber;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.DifferentialVelocityDutyCycle;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealGrabber extends Grabber {

  private final SparkMax motor;
  private final DigitalInput detectSensor;
  private final DigitalInput clearSensor;

  private Debouncer algaeDebouncer = new Debouncer(0.3);
  private Debouncer coralDebouncer = new Debouncer(0.15);

  private boolean hasCoral = false;
  private boolean coralClear = false;

  private boolean detectedAlgae = false;

  public RealGrabber() {
    motor = new SparkMax(CAN.MANIPULATOR_GRABBER, MotorType.kBrushless);

    Logger.logBoolean(getName() + "/detectedAlgae", this::detectedAlgae);
    Logger.logBoolean(getName() + "/coralClear", this::coralClear);
    Logger.logNumber(getName() + "/vel", () -> motor.getEncoder().getVelocity());
    Logger.logNumber(getName() + "/get", () -> motor.get());
    Logger.logNumber(getName() + "/amps", () -> motor.getOutputCurrent());
    Logger.logNumber(getName() + "/temp", () -> motor.getMotorTemperature());


    SparkMaxConfig config = new SparkMaxConfig();

    SparkMaxUtil.configure(config, false, IdleMode.kBrake);
    SparkMaxUtil.saveAndLog(this, motor, config);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    detectSensor = new DigitalInput(DIO.CORAL_DETECT_BEAM_BREAK);
    clearSensor = new DigitalInput(DIO.CORAL_CLEAR_BEAM_BREAK);

    setDefaultCommand(hold());

    StatusChecks.under(this).add("motor", motor);
  }

  public Command runSpeedOnce(double speed) {
    return this.runOnce(() -> motor.set(speed));
  }

  public Command runSpeed(double speed) {
    return this.run(() -> motor.set(speed));
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  public boolean coralClear() {
    return coralClear;
  }

  @Override
  public Command intakeCoral() {
    return Commands.sequence(
        runSpeed(MANIPULATOR.CORAL_IN_SPEED).until(this::hasCoral),
        runSpeed(MANIPULATOR.CORAL_SLOW_IN_SPEED).until(this::coralClear),
        stopOnce()
        );
  }

  @Override
  public Command dropCoral() {
    return runSpeed(MANIPULATOR.CORAL_OUT_SPEED)
        .withDeadline(
            Commands.sequence(
                Commands.waitUntil(this::hasCoral), Commands.waitUntil(() -> !hasCoral())));
  }

  @Override
  public Command adjustCoral() {
    return runSpeed(MANIPULATOR.CORAL_ADJUST_SPEED);
  }

  public boolean detectedAlgae() {
    return detectedAlgae;
  }

  @Override
  public Command intakeAlgae() {
    return runSpeed(MANIPULATOR.ALGAE_IN_SPEED)
        .until(this::detectedAlgae)
        .andThen(Commands.print("FINISHED INTAKING"))
        .finallyDo(() -> expectAlgae(detectedAlgae()));
  }

  @Override
  public Command dropAlgae() {
    return runSpeed(MANIPULATOR.ALGAE_OUT_SPEED).finallyDo(() -> expectAlgae(false));
  }

  @Override
  public Command forwards() {
    return runSpeed(MANIPULATOR.BASE_SPEED);
  }

  @Override
  public Command backwards() {
    return runSpeed(-MANIPULATOR.BASE_SPEED);
  }

  @Override
  public Command holdAlgae() {
    if (!MANIPULATOR.ALGAE_GRIP_CHECK_ENABLED) return stop();
    return Commands.sequence(
            runSpeed(MANIPULATOR.ALGAE_GRIP_CHECK_SPEED)
                .until(this::detectedAlgae)
                .withTimeout(MANIPULATOR.ALGAE_GRIP_CHECK_TIME)
                .finallyDo(() -> expectAlgae(detectedAlgae())),
            runSpeed(MANIPULATOR.ALGAE_HOLD_SPEED).withTimeout(MANIPULATOR.ALGAE_GRIP_IDLE_TIME));
  }

  // update this for both game pieces
  @Override
  public Command hold() {
    return Commands.either(stop(), holdAlgae(), () -> hasCoral() || !hasAlgae()).repeatedly();
  }

  public Command stopOnce() {
    return runSpeedOnce(0.0);
  }

  @Override
  public Command stop() {
    return runSpeed(0);
  }

  public boolean isStalled() {
    return Math.abs(motor.get()) > 0.0 && Math.abs(motor.getEncoder().getVelocity()) < 3000;
  }

  @Override
  public void periodic() {
    super.periodic();

    detectedAlgae = algaeDebouncer.calculate(isStalled());
    hasCoral = coralDebouncer.calculate(!detectSensor.get());
    coralClear = clearSensor.get();
  }
}
