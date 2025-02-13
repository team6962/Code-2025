// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.team6962.lib.swerve.SwerveDrive;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Preferences.ELEVATOR;
import frc.robot.subsystems.elevator.Elevator;

/** An example command that uses an example subsystem. */
public class SlowDrivetrain extends Command {
  private final SwerveDrive swerveDrive;
  private final Elevator elevator;

  public SlowDrivetrain(SwerveDrive swerveDrive, Elevator elevator) {
    this.swerveDrive = swerveDrive;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinearVelocity maxSpeed = swerveDrive.getConstants().maxDriveSpeed();
    Double scaleFactor = (elevator.getAverageHeight().div(elevator.getMaxHeight())).magnitude();
    if (elevator.getAverageHeight().lt(ELEVATOR.MAX_UNLIMITED_HEIGHT)) {
      scaleFactor = 1.0;
    }
    swerveDrive.limitSpeed(maxSpeed.times(scaleFactor));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.limitSpeed(swerveDrive.getConstants().maxDriveSpeed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
