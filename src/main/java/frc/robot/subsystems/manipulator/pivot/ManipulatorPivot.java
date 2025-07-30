package frc.robot.subsystems.manipulator.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.temp.Constants.ENABLED_SYSTEMS;
import java.util.function.Supplier;

public interface ManipulatorPivot extends Subsystem {
  public Angle getAngle();

  public Command pivotTo(Supplier<Angle> angleSupplier, Angle tolerance);

  public Command pivotTo(Supplier<Angle> angleSupplier);

  public Command hold();

  public Command coralIntake();

  public Command coralL1();

  public Command coralL23();

  public Command coralL4();

  public Command algaeReef();

  public Command algaeBargeSetup();

  public Command algaeBargeShoot();

  public Command algaeProcessor();

  public Command algaeGround();

  public Command stow();

  public Command safe(Angle tolerance);

  public Command safe();

  public Command stop();

  public Command up();

  public Command down();

  public boolean doneMoving();

  public boolean inRange(Angle angle);

  public void setMinMaxAngle(Angle min, Angle max);

  public static ManipulatorPivot create() {
    if (ENABLED_SYSTEMS.isManipulatorEnabled()) return new RealManipulatorPivot();
    else return new DisabledManipulatorPivot();
  }
}
