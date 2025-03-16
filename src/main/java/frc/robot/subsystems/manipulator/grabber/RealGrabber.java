package frc.robot.subsystems.manipulator.grabber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealGrabber extends Grabber {

  private final SparkMax motor;
  private final DigitalInput sensor;

  private Debouncer algaeDebouncer;
  private Debouncer coralDebouncer = new Debouncer(0.10);

  private boolean hasCoral = false;

  private boolean detectedAlgae = false;

  private Timer gripCheckTimer = new Timer();

  public RealGrabber() {
    motor = new SparkMax(CAN.MANIPULATOR_GRABBER, MotorType.kBrushless);

    Logger.logBoolean(getName() + "/motorsStalled", () -> detectedAlgae);
    Logger.logNumber(getName() + "/gripCheckTime", () -> gripCheckTimer.get());

    resetDebouncer();

    SparkMaxConfig config = new SparkMaxConfig();

    SparkMaxUtil.configure(config, false, IdleMode.kBrake);
    SparkMaxUtil.saveAndLog(this, motor, config);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sensor = new DigitalInput(DIO.CORAL_BEAM_BREAK);

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

  public Command intakeCoral() {
    return runSpeed(MANIPULATOR.CORAL_IN_SPEED).until(this::hasCoral);
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

  public Command intakeAlgae() {
    return runSpeed(MANIPULATOR.ALGAE_IN_SPEED)
        .until(this::hasAlgae)
        .andThen(() -> expectAlgae(true));
  }

  public Command dropAlgae() {
    return runSpeed(MANIPULATOR.ALGAE_OUT_SPEED)
        .finallyDo(
            () -> {
              expectAlgae(false);
              stop();
            });
  }

  public Command checkAlgaeGrip() {
    return Commands.runOnce(this::resetDebouncer)
        .andThen(
            Commands.parallel(
                runSpeed(MANIPULATOR.ALGAE_GRIP_CHECK_SPEED),
                Commands.race(
                    Commands.waitUntil(() -> detectedAlgae).andThen(() -> expectAlgae(true)),
                    Commands.waitSeconds(MANIPULATOR.ALGAE_GRIP_CHECK_TIME.in(Seconds))
                        .andThen(() -> expectAlgae(false)))));
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

  // update this for both game pieces
  public Command hold() {
    return run(
        () -> {
          if (!ENABLED_SYSTEMS.MANIPULATOR) motor.set(0);
          else if (hasCoral()) motor.set(MANIPULATOR.CORAL_HOLD_SPEED);
          else if (!hasAlgae()) motor.set(0);
          else if (MANIPULATOR.ALGAE_GRIP_CHECK_ENABLED
              && gripCheckTimer.advanceIfElapsed(MANIPULATOR.ALGAE_GRIP_CHECK_RATE.in(Seconds))) {
            checkAlgaeGrip().schedule();
          } else motor.set(0);
        });
  }

  public Command stop() {
    return runSpeed(0);
  }

  @Override
  public void periodic() {
    super.periodic();

    detectedAlgae = algaeDebouncer.calculate(Amps.of(motor.getOutputCurrent()).gt(Amps.of(40)));
    hasCoral = coralDebouncer.calculate(!sensor.get());
  }
}
