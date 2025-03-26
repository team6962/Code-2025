package frc.robot.auto.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.telemetry.Logger;
import com.team6962.lib.utils.CommandUtils;
import com.team6962.lib.utils.KinematicsUtils;
import com.team6962.lib.utils.MeasureMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.Constants.Field.CoralStation;
import frc.robot.Constants.ReefPositioning;
import frc.robot.Constants.StationPositioning;
import frc.robot.auto.utils.AutoPaths.CoralPosition;
import frc.robot.commands.PieceCombos;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;

public class AutoCommands {
    private SwerveDrive swerveDrive;
    private Manipulator manipulator;
    private Elevator elevator;
    private PieceCombos pieceCombos;

    public AutoCommands(
        SwerveDrive swerveDrive,
        Manipulator manipulator,
        Elevator elevator,
        PieceCombos pieceCombos
    ) {
        this.swerveDrive = swerveDrive;
        this.manipulator = manipulator;
        this.elevator = elevator;
        this.pieceCombos = pieceCombos;
    }

  public static enum PolePattern {
        LEFT(1, 2),
        RIGHT(0, 2),
        ALL(0, 1);

        public final int start;
        public final int increment;

        private PolePattern(int start, int increment) {
            this.start = start;
            this.increment = increment;
        }
  }

    public Subsystem[] getRequirements() {
        return new Subsystem[] {
            manipulator.pivot, manipulator.grabber, manipulator.funnel, elevator, swerveDrive
        };
    }

    // Full Autonomous

    public Command intakeCoral(CoralStation station) {
        Pose2d intake =
            StationPositioning.getNearestIntakePose(station, swerveDrive.getEstimatedPose());

        // Drives to the coral station to pickup coral, finishing either when
        // a coral is detected or the robot gets to the intake pose and the
        // human player should have dropped the coral into the funnel, where
        // there is no beam break sensor.
        return Commands.race(
            Commands.sequence(
                swerveDrive.pathfindTo(intake),
                swerveDrive.alignTo(intake).withTimeout(Seconds.of(0.1))
            ),
            pieceCombos.intakeCoral()
        );
    }

    private List<Boolean> commandsRunning;

    private Command annotate(String name, Command command) {
        return command.deadlineFor(new Command() {
            int commandIndex = 0;

            @Override
            public void initialize() {
                for (int i = 0; i < commandsRunning.size(); i++) {
                    if (!commandsRunning.get(i)) {
                        commandsRunning.set(i, true);
                        commandIndex = i;
                        return;
                    }
                }

                commandIndex = commandsRunning.size();
                commandsRunning.add(true);

                Logger.log("AutoCommands/activeCommands/" + commandIndex, name);
            }

            @Override
            public void end(boolean interrupted) {
                commandsRunning.set(commandIndex, false);
                Logger.log("AutoCommands/activeCommands/" + commandIndex, "");
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        });
    }

    public Command placeCoral(CoralPosition position) {
        Pose2d alignPose = ReefPositioning.getCoralAlignPose(position.pole);
        Pose2d placePose = ReefPositioning.getCoralPlacePose(position.pole);

        return Commands.sequence(
            // Drive to the coral pole with Pathplanner, moving the elevator and
            // manipulator early if possible.
            Commands.deadline(
                // Pathfind to the alignment pose
                annotate("pathfind align", swerveDrive.pathfindTo(alignPose)),
                // Run a sequence of movements to prepare the elevator and
                // manipulator early, that will immediately end when the robot
                // gets to the alignment pose.
                Commands.sequence(
                    // Intake coral if we haven't got already. This is for if
                    // the preloaded coral isn't fully inside the manipulator.
                    CommandUtils.onlyIf(
                        () -> !manipulator.grabber.hasCoral() || !manipulator.grabber.isCoralClear(),
                        annotate("intake coral", pieceCombos.intakeCoral())
                    ),
                    // Move the elevator at maximum speed until it gets to the
                    // maximum safe height to drive at full speed without
                    // tipping over.
                    annotate("towards ready", Commands.parallel(
                        elevator.move(1.0).until(() -> elevator.getAverageHeight().gt(ELEVATOR.AUTO.READY_HEIGHT)),
                        manipulator.pivot.hold(),
                        manipulator.grabber.stop()
                    )),
                    // Finely position the elevator at the maximum safe height,
                    // then keep it there until the robot is near to the
                    // alignment pose and slow enough to raise the elevator higher.
                    Commands.sequence(
                        annotate("to ready", pieceCombos.readyL3()),
                        annotate("hold ready", pieceCombos.holdCoral())
                    ).until(() ->
                        swerveDrive.getEstimatedPose().getTranslation().getDistance(alignPose.getTranslation()) < 2.0 &&
                        KinematicsUtils.getTranslation(swerveDrive.getEstimatedSpeeds()).getNorm() < 3.0
                    ),
                    // Move the elevator to the maximum height to prepare for
                    // placing, then hold it there. This command is wrapped in a
                    // deadline so this sequence will end if the robot when to
                    // the reef pole.
                    annotate("move l4", pieceCombos.coralL4()),
                    annotate("hold l4", pieceCombos.holdCoral())
                )
            ),
            // In case the robot got to the coral pole before it was done intake,
            // continue intaking until a coral is in the manipulator and clear of
            // the funnel.
            CommandUtils.onlyIf(
                () -> !manipulator.grabber.hasCoral() || !manipulator.grabber.isCoralClear(),
                annotate("intake coral", pieceCombos.intakeCoral())
            ).withTimeout(1.0),
            // If the robot has coral, align to the place pose, raise the
            // elevator, and drop it. Otherwise, end the command, skipping this
            // pole.
            CommandUtils.onlyIf(
                () -> manipulator.grabber.hasCoral() && manipulator.grabber.isCoralClear(),
                Commands.sequence(
                    // Move the elevator and manipulator to the L4 placing
                    // position, aligning at the same time.
                    Commands.deadline(
                        pieceCombos.coralL4(),
                        swerveDrive
                            .alignTo(placePose, Inches.of(1), Degrees.of(4))
                            .withEndWithinTolerance(false)),
                    // Finish aligning while holding the elevator and
                    // manipulator in the same place.
                    Commands.deadline(
                        swerveDrive.alignTo(placePose, Inches.of(1), Degrees.of(4)),
                        pieceCombos.holdCoral()
                    ),
                    // Drop the coral while keeping the elevator and manipulator
                    // in place.
                    manipulator
                        .grabber
                        .dropCoral()
                        .deadlineFor(manipulator.pivot.hold(), elevator.hold()),
                    // Move the elevator down at maximum speed until it gets
                    // under the safe height to drive at.
                    Commands.parallel(
                        elevator.move(-1.0),
                        manipulator.pivot.hold(),
                        manipulator.grabber.stop()
                    ).until(() -> elevator.getAverageHeight().lt(ELEVATOR.AUTO.READY_HEIGHT))
                )
            )
        );
    }

  // Autonomous Commands in Teleop

  private int getClosestReefPole(Pose2d pose, PolePattern pattern) {
    int closestPole = 0;
    double closestDistance = Double.MAX_VALUE;

    for (int i = pattern.start; i < 12; i += pattern.increment) {
      Translation2d pole = ReefPositioning.getCoralAlignPose(i).getTranslation();

      if (pose.getTranslation().getDistance(pole) < closestDistance) {
        closestPole = i;
        closestDistance = pose.getTranslation().getDistance(pole);
      }
    }

    return closestPole;
  }

  public int getClosestReefFace(Pose2d pose) {
    int closestFace = 0;
    double closestDistance = Double.MAX_VALUE;

    for (int i = 0; i < 6; i++) {
      Translation2d face = ReefPositioning.getAlgaeAlignPose(i).getTranslation();

      if (pose.getTranslation().getDistance(face) < closestDistance) {
        closestFace = i;
        closestDistance = pose.getTranslation().getDistance(face);
      }
    }

    return closestFace;
  }

  public Command alignPole(int pole, boolean endWithinTolerance, boolean useAlignPose) {
    return swerveDrive
        .pathfindTo(
            useAlignPose
                ? ReefPositioning.getCoralAlignPose(pole)
                : ReefPositioning.getCoralPlacePose(pole))
        .andThen(
            swerveDrive
                .alignTo(ReefPositioning.getCoralPlacePose(pole), Inches.of(0.5), Degrees.of(2))
                .withEndWithinTolerance(endWithinTolerance));
  }

  public Command alignFace(int face, boolean endWithinTolerance) {
    return swerveDrive
        .pathfindTo(ReefPositioning.getAlgaeAlignPose(face))
        .andThen(
            swerveDrive
                .alignTo(ReefPositioning.getAlgaePlacePose(face))
                .withEndWithinTolerance(endWithinTolerance));
  }

  public Command alignToClosestPoleTeleop(PolePattern pattern, Supplier<Command> rumble) {
    return Commands.defer(
        () -> {
          int pole = getClosestReefPole(swerveDrive.getEstimatedPose(), pattern);
          Pose2d polePose = ReefPositioning.getCoralPlacePose(pole);

          return alignPole(pole, false, false)
              .alongWith(
                  Commands.waitUntil(
                          () ->
                              polePose
                                          .getTranslation()
                                          .getDistance(
                                              swerveDrive.getEstimatedPose().getTranslation())
                                      < Units.inchesToMeters(1)
                                  && MeasureMath.minAbsDifference(
                                              polePose.getRotation(),
                                              swerveDrive.getEstimatedHeading())
                                          .getDegrees()
                                      < 3)
                      .andThen(rumble.get()));
        },
        Set.of(swerveDrive.useMotion()));
  }

  public Command alignToClosestFaceTeleop() {
    return Commands.defer(
        () -> alignFace(getClosestReefFace(swerveDrive.getEstimatedPose()), false),
        Set.of(swerveDrive.useMotion()));
  }
}
