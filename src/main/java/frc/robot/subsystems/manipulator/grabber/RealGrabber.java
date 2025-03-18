package frc.robot.subsystems.manipulator.grabber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.pathplanner.lib.auto.CommandUtil;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.telemetry.StatusChecks;
import com.team6962.lib.utils.CommandUtils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.MANIPULATOR;
import frc.robot.Constants.Constants.MANIPULATOR_PIVOT;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealGrabber extends Grabber {

  private final SparkMax motor;
  private final DigitalInput detectSensor;
  private final DigitalInput clearSensor;

  private Debouncer algaeDebouncer;
  private Debouncer coralDebouncer = new Debouncer(0.15);

  private boolean hasCoral = false;
<<<<<<< HEAD
  private boolean coralClear = false;

=======
>>>>>>> 4b37b57 (reimplement algae hold)
  private boolean detectedAlgae = false;


  private Timer gripCheckTimer = new Timer();

  public RealGrabber() {
    motor = new SparkMax(CAN.MANIPULATOR_GRABBER, MotorType.kBrushless);

    Logger.logBoolean(getName() + "/motorsStalled", () -> detectedAlgae);
    Logger.logNumber(getName() + "/amps", () -> motor.getOutputCurrent());
    Logger.logNumber(getName() + "/temp", () -> motor.getMotorTemperature());
    Logger.logNumber(getName() + "/gripCheckTime", () -> gripCheckTimer.get());

    resetDebouncer();

    SparkMaxConfig config = new SparkMaxConfig();

    SparkMaxUtil.configure(config, false, IdleMode.kBrake);
    SparkMaxUtil.saveAndLog(this, motor, config);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    detectSensor = new DigitalInput(DIO.CORAL_DETECT_BEAM_BREAK);
    clearSensor = new DigitalInput(DIO.CORAL_CLEAR_BEAM_BREAK);

    setDefaultCommand(hold());

    StatusChecks.under(this).add("motor", motor);
  }

  public Command runSpeed(double speed) {
    return this.run(() -> motor.set(speed));
  }

  @Override
  public boolean hasCoral() {
    return hasCoral;
  }

  public boolean isClear() {
    return coralClear;
  }

  public Command intakeCoral() {
    return Commands.sequence(
      runSpeed(MANIPULATOR.CORAL_IN_SPEED).until(() -> !isClear()),
      runSpeed(MANIPULATOR.CORAL_IN_SPEED).until(this::hasCoral),
      runSpeed(MANIPULATOR.CORAL_SLOW_IN_SPEED).until(this::isClear),
      stop()
    );
  }

  public Command dropCoral() {
    return runSpeed(MANIPULATOR.CORAL_OUT_SPEED)
        .withDeadline(
            Commands.sequence(
                Commands.waitUntil(this::hasCoral), Commands.waitUntil(() -> !hasCoral())));
  }

  public Command adjustCoral() {
    return runSpeed(MANIPULATOR.CORAL_ADJUST_SPEED);
  }

  public boolean detectedAlgae() {
    return detectedAlgae;
  }

  public Command intakeAlgae() {
    return runSpeed(MANIPULATOR.ALGAE_IN_SPEED)
        .until(this::detectedAlgae)
        .finallyDo((b) -> expectAlgae(b));
  }

  public Command dropAlgae() {
    return runSpeed(MANIPULATOR.ALGAE_OUT_SPEED)
        .finallyDo(() -> expectAlgae(false));
  }

  public Command checkAlgaeGrip() {
    return Commands.sequence(
      Commands.runOnce(this::resetDebouncer),
      Commands.parallel(
                runSpeed(MANIPULATOR.ALGAE_GRIP_CHECK_SPEED),
                Commands.race(
                    Commands.waitUntil(this::detectedAlgae).andThen(() -> expectAlgae(true)),
                    Commands.waitTime(MANIPULATOR.ALGAE_GRIP_CHECK_TIME).andThen(() -> expectAlgae(false))))
    );
  }

  private void resetDebouncer() {
    algaeDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  }

  public Command forwards() {
    return runSpeed(MANIPULATOR.BASE_SPEED);
  }

  public Command backwards() {
    return runSpeed(-MANIPULATOR.BASE_SPEED);
  }

  public Command holdAlgae() {
    if (!MANIPULATOR.ALGAE_GRIP_CHECK_ENABLED) return stop();
    return Commands.sequence(
      runSpeed(MANIPULATOR.ALGAE_GRIP_CHECK_SPEED).until(this::detectedAlgae),
      stop().withTimeout(MANIPULATOR.ALGAE_GRIP_CHECK_TIME)
    ).repeatedly();
  }

  // update this for both game pieces
  public Command hold() {
    return Commands.either(stop(), holdAlgae(), () -> hasCoral() || !hasAlgae());
  }

  public Command stop() {
    return runSpeed(0);
  }

  @Override
  public void periodic() {
    super.periodic();

    detectedAlgae = algaeDebouncer.calculate(Amps.of(motor.getOutputCurrent()).gt(Amps.of(40)));
    hasCoral = coralDebouncer.calculate(!detectSensor.get());
    coralClear = clearSensor.get();
  }
}
