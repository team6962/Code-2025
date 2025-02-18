package frc.robot.subsystems.hang;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.util.hardware.motion.PivotController;
import frc.robot.Constants.Constants.HANG_PIVOT;
import frc.robot.Constants.Constants.VOLTAGE_LADDER;
import frc.robot.RobotContainer;

public class Hang extends PivotController {

  public Hang() {
    super(
        CAN.HANG,
        DIO.HANG_ENCODER,
        HANG_PIVOT.ENCODER_OFFSET.in(Rotations),
        HANG_PIVOT.PROFILE.kP,
        HANG_PIVOT.PROFILE.kI,
        HANG_PIVOT.PROFILE.kD,
        HANG_PIVOT.PROFILE.kS,
        HANG_PIVOT.GEARING,
        HANG_PIVOT.MIN_ANGLE,
        HANG_PIVOT.MAX_ANGLE,
        Degrees.of(2.0),
        false);
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.HANG) stopMotor();
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.HANG) stopMotor();
  }

  public Command deploy() {
    return setTargetAngleCommand(HANG_PIVOT.HANG_ANGLE);
  }

  public Command stow() {
    return setTargetAngleCommand(HANG_PIVOT.STOW_ANGLE);
  }

  public Command stop() {
    return Commands.run(this::stopMotor);
  }

  public Command test() {
    return Commands.sequence(deploy(), stow());
  }

  public Command setTargetAngleCommand(Angle angle) {
    if (!ENABLED_SYSTEMS.HANG) return stop();
    return this.run(() -> moveTowards(angle)).until(this::doneMoving);
  }

  @Override
  public void simulationPeriodic() {}
}
