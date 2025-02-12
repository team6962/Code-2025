// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.StatusChecks;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

/** An example command that uses an example subsystem. */
public class PrematchChecks extends SequentialCommandGroup {
  private final SwerveDrive swerveDrive;
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Hang hang;
  private final Intake intake;

  public PrematchChecks(
    SwerveDrive swerveDrive,
    Elevator elevator,
    Manipulator manipulator,
    Hang hang,
    Intake intake
  ) {
    this.swerveDrive = swerveDrive;
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.hang = hang;
    this.intake = intake;

    addCommands(
        elevator.test(),
        manipulator.test(),
        hang.test(),
        Commands.runOnce(StatusChecks::refresh)
    );
  }
}
