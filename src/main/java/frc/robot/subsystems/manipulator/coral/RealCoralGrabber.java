package frc.robot.subsystems.manipulator.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team6962.lib.telemetry.StatusChecks;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.MANIPULATOR;
import frc.robot.util.hardware.SparkMaxUtil;

public class RealCoralGrabber extends CoralGrabber {
  private final SparkMax motor;
  private final DigitalInput sensor;
  private boolean detectsGamePiece = false;

  public RealCoralGrabber() {
    motor = new SparkMax(CAN.MANIPULATOR_CORAL, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();

    SparkMaxUtil.configure(config, false, IdleMode.kBrake);
    SparkMaxUtil.saveAndLog(this, motor, config);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sensor = new DigitalInput(DIO.CORAL_BEAM_BREAK);

    setDefaultCommand(hold());

    StatusChecks.under(this).add("motor", motor);
  }

  @Override
  public boolean hasGamePiece() {
    return detectsGamePiece;
  }

  public Command runSpeed(double speed) {
    return this.run(() -> motor.set(speed));
  }

  public Command intake() {
    return runSpeed(MANIPULATOR.CORAL_IN_SPEED).until(this::hasGamePiece);
    // .andThen(runSpeed(MANIPULATOR.CORAL_ADJUST_SPEED)
    //   .withDeadline(Commands.waitTime(MANIPULATOR.CORAL_ADJUST_TIME)));
  }

  public Command drop() {
    return runSpeed(MANIPULATOR.CORAL_OUT_SPEED)
        .withDeadline(
            Commands.sequence(
                Commands.waitUntil(this::hasGamePiece), Commands.waitUntil(() -> !hasGamePiece())));
  }

  public Command forwards() {
    return runSpeed(MANIPULATOR.CORAL_IN_SPEED);
  }

  public Command backwards() {
    return runSpeed(-MANIPULATOR.CORAL_IN_SPEED);
  }

  public Command hold() {
    return run(
        () -> {
          if (hasGamePiece()) motor.set(MANIPULATOR.CORAL_HOLD_SPEED);
          else motor.set(0);
        });
  }

  public Command stop() {
    return runSpeed(0);
  }

  @Override
  public void periodic() {
    super.periodic();

    detectsGamePiece = !sensor.get();
  }
}
