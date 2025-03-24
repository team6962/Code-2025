package frc.robot.auto.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.SwerveDrive;
import com.team6962.lib.utils.CommandUtils;
import com.team6962.lib.utils.MeasureMath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Constants.ELEVATOR;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.SWERVE;
import frc.robot.Constants.Field;
import frc.robot.Constants.Field.CoralStation;
import frc.robot.Constants.ReefPositioning;
import frc.robot.Constants.StationPositioning;
import frc.robot.auto.utils.AutoPaths.CoralPosition;
import frc.robot.commands.PieceCombos;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Algae;
import java.util.Set;
import java.util.function.Supplier;

public class AutonomousCommands {
  private SwerveDrive swerveDrive;
  private Manipulator manipulator;
  private Elevator elevator;
  private PieceCombos pieceCombos;
  private Subsystem[] subsystems;

  public AutonomousCommands(
      SwerveDrive swerveDrive,
      Manipulator manipulator,
      Elevator elevator,
      PieceCombos pieceCombos) {
    this.swerveDrive = swerveDrive;
    this.manipulator = manipulator;
    this.elevator = elevator;
    this.pieceCombos = pieceCombos;
  }

  public static enum PolePattern {
    LEFT(1, 2),
    RIGHT(0, 2),
    CLOSEST(0, 1);

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

  public Command intakeCoral(CoralStation station) {
    Pose2d intake =
        StationPositioning.getNearestIntakePose(station, swerveDrive.getEstimatedPose());

    return Commands.race(
        // swerveDrive.pathfindTo(intake),
        Commands.sequence(
            swerveDrive.pathfindTo(intake),
            swerveDrive.alignTo(intake).withTimeout(Seconds.of(0.1))),
        pieceCombos.intakeCoral()); // .until(() -> !manipulator.grabber.isCoralClear());

    // return Commands.sequence(
    //     Commands.parallel(
    //         swerveDrive.pathfindTo(intake),
    //         CommandUtils.selectByMode(
    //             pieceCombos.intakeCoral(),
    //             CommandUtils.simulationMessage(
    //                 "Moving elevator and manipulator for coral intaking", 0.5))),
    //     Commands.parallel(
    //       pieceCombos.intakeCoral(),
    //       swerveDrive.alignTo(intake)
    //     ).until(() -> !manipulator.grabber.isCoralClear())
    // );
  }

  public Command processAlgae() {
    Pose2d processorPose =
        new Pose2d(
            Units.inchesToMeters(235.726104),
            SWERVE.CONFIG.chassis().outerWidth().div(2).plus(Inches.of(18)).in(Meters),
            Rotation2d.fromDegrees(-90));

    Pose2d processorNewPose =
        Field.getProcessorPose()
            .plus(
                new Transform2d(
                    new Translation2d(
                        0,
                        swerveDrive
                            .getConstants()
                            .chassis()
                            .outerLength()
                            .div(2)
                            .plus(Inches.of(18))
                            .in(Meters)),
                    new Rotation2d()));

    processorNewPose = new Pose2d(processorNewPose.getTranslation(), Rotation2d.fromDegrees(-90));

    return Commands.sequence(
        Commands.parallel(
            CommandUtils.selectByMode(
                pieceCombos.algaeProcessor(),
                CommandUtils.simulationMessage(
                    "Moving elevator and manipulator for algae processor", 0.5)),
            swerveDrive.pathfindTo(processorNewPose)),
        swerveDrive.alignTo(processorNewPose),
        CommandUtils.selectByMode(
            manipulator.grabber.dropAlgae(), CommandUtils.simulationMessage("Dropping algae", 0.5)),
        swerveDrive.alignTo(
            new Pose2d(
                Units.inchesToMeters(235.726104),
                SWERVE.CONFIG.chassis().outerWidth().div(2).plus(Inches.of(24)).in(Meters),
                Rotation2d.fromDegrees(-90)),
            Inches.of(4),
            Degrees.of(45)));
  }

  public Command driveToProcessor() {
    return Commands.sequence(
        CommandUtils.selectByMode(
            pieceCombos.algaeProcessor(),
            CommandUtils.simulationMessage("Preparing for algae processor", 0.5)),
        swerveDrive.pathfindTo(
            new Pose2d(
                Units.inchesToMeters(235.726104),
                SWERVE.CONFIG.chassis().outerWidth().div(2).plus(Inches.of(18)).in(Meters),
                Rotation2d.fromDegrees(-90))),
        CommandUtils.selectByMode(
            manipulator.grabber.dropAlgae(),
            CommandUtils.simulationMessage("Dropping algae", 0.25)));
  }

  public Command alignCoral(int pole, boolean endWithinTolerance, boolean useAlignPose) {
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

  public Command alignAlgae(int face, boolean endWithinTolerance) {
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

          return alignCoral(pole, false, false)
              .alongWith(
                  Commands.waitUntil(
                          () ->
                              polePose
                                          .getTranslation()
                                          .getDistance(
                                              swerveDrive.getEstimatedPose().getTranslation())
                                      < Units.inchesToMeters(1)
                                  && MeasureMath.minDifference(
                                              polePose.getRotation(),
                                              swerveDrive.getEstimatedHeading())
                                          .getDegrees()
                                      < 3)
                      .andThen(rumble.get()));
        },
        Set.of(swerveDrive.useMotion()));
  }

  public Command alignToClosestPole(
      PolePattern pattern, boolean endWithinTolerance, boolean useAlignPose) {
    return Commands.defer(
        () ->
            alignCoral(
                getClosestReefPole(swerveDrive.getEstimatedPose(), pattern),
                endWithinTolerance,
                useAlignPose),
        Set.of(swerveDrive.useMotion()));
  }

  public Command alignToClosestFace(boolean endWithinTolerance) {
    return Commands.defer(
        () -> alignAlgae(getClosestReefFace(swerveDrive.getEstimatedPose()), endWithinTolerance),
        Set.of(swerveDrive.useMotion()));
  }

  public Command driveToPole(int pole) {
    return swerveDrive
        .pathfindTo(ReefPositioning.getCoralPlacePose(pole))
        .andThen(swerveDrive.alignTo(ReefPositioning.getCoralPlacePose(pole)));
  }

  public Command placeCoral(CoralPosition position) {
    Pose2d alignPose = ReefPositioning.getCoralAlignPose(position.pole);
    Pose2d placePose = ReefPositioning.getCoralPlacePose(position.pole);

    return Commands.sequence(
        // CommandUtils.selectByMode(
        //         pieceCombos.stow()
        //           .deadlineFor(manipulator.grabber.hold())
        //           .andThen(elevator.hold()),
        //         CommandUtils.simulationMessage("Stowing elevator", position.level * 0.5))
        //
        // .withDeadline(swerveDrive.pathfindTo(ReefPositioning.getCoralAlignPose(position.pole))),
        Commands.deadline(
            swerveDrive.pathfindTo(alignPose),
            Commands.sequence(
                    pieceCombos.intakeCoral(),
                    pieceCombos.readyL3(),
                    pieceCombos.holdCoral()
                .until(
                    () ->
                        swerveDrive
                                    .getEstimatedPose()
                                    .getTranslation()
                                    .getDistance(alignPose.getTranslation())
                                < 2.0
                            && Math.hypot(
                                    swerveDrive.getEstimatedSpeeds().vxMetersPerSecond,
                                    swerveDrive.getEstimatedSpeeds().vyMetersPerSecond)
                                < 3.0)
                .andThen(pieceCombos.coralL4())
            // Commands.sequence(
            //   pieceCombos.stow()
            //     .deadlineFor(manipulator.grabber.hold())
            //     .andThen(elevator.hold())
            //     .until(() ->
            //
            // swerveDrive.getEstimatedPose().getTranslation().getDistance(alignPose.getTranslation()) < 2.0 &&
            //       Math.hypot(swerveDrive.getEstimatedSpeeds().vxMetersPerSecond,
            // swerveDrive.getEstimatedSpeeds().vyMetersPerSecond) < 3.0
            //     ),
            //   manipulator.grabber.hold()
            //     .withDeadline(pieceCombos.coral(position.level))
            // )
            )
        ),
        pieceCombos.intakeCoral()
          .onlyIf(() -> !manipulator.grabber.hasCoral() || !manipulator.grabber.isCoralClear()),
        Commands.sequence(
                Commands.parallel(
                    swerveDrive.alignTo(placePose, Inches.of(1), Degrees.of(4)),
                    Commands.sequence(
                        pieceCombos.coralL4(),
                        pieceCombos
                            .holdCoral()
                            .until(
                                () ->
                                    swerveDrive.isWithinToleranceOf(
                                        placePose, Inches.of(1), Degrees.of(4)))
                        // () ->
                        // swerveDrive.getEstimatedPose().getTranslation().getDistance(placePose.getTranslation()) < Units.inchesToMeters(0.5) &&
                        // Math.abs(MeasureMath.minDifference(swerveDrive.getEstimatedPose().getRotation(), placePose.getRotation()).getDegrees()) < 2.0
                        // )
                        )),
                // swerveDrive.alignTo(ReefPositioning.getCoralPlacePose(position.pole),
                // Inches.of(0.5), Degrees.of(2))
                //   .deadlineFor(manipulator.grabber.hold(), manipulator.pivot.hold(),
                // elevator.hold()),
                manipulator
                    .grabber
                    .dropCoral()
                    .deadlineFor(manipulator.pivot.hold(), elevator.hold()),
                pieceCombos
                    .stow()
                    .until(() -> elevator.getAverageHeight().lt(ELEVATOR.AUTO.READY_HEIGHT)))
            .onlyIf(() -> manipulator.grabber.hasCoral() || !manipulator.grabber.isCoralClear()));
  }

  public Command driveToAlgae(int face) {
    return swerveDrive
        .pathfindTo(ReefPositioning.getAlgaeAlignPose(face))
        .andThen(swerveDrive.alignTo(ReefPositioning.getAlgaePlacePose(face)));
  }

  public Command pickupAlgae(int face, int level) {
    return Commands.sequence(
        swerveDrive.pathfindTo(ReefPositioning.getAlgaeAlignPose(face)),
        CommandUtils.selectByMode(
            pieceCombos.algae(level),
            CommandUtils.simulationMessage("Moving to level " + level, level * 0.5)),
        swerveDrive.alignTo(ReefPositioning.getAlgaePlacePose(face)),
        CommandUtils.selectByMode(
            manipulator.grabber.intakeAlgae(),
            CommandUtils.simulationMessage("Intaking algae", 0.5)),
        swerveDrive.alignTo(ReefPositioning.getAlgaeLeavePose(face), Inches.of(3), Degrees.of(45)),
        CommandUtils.selectByMode(
            pieceCombos.stow(), CommandUtils.simulationMessage("Stowing elevator", level * 0.5)));
  }

  public Command pathfindToPole(int poleNumber) {
    return swerveDrive
        .pathfindTo(ReefPositioning.getCoralAlignPose(poleNumber))
        .andThen(swerveDrive.alignTo(ReefPositioning.getCoralPlacePose(poleNumber)));
  }

  // private static Translation2d ALGAE_SETUP = new Translation2d(1.98, 2.18);
  // private static Translation2d ALGAE_DRIVE_OVER = new Translation2d(1.21, 2.18);

  final double BARGE_X = 7.75;
  final double BARGE_MIN = 4.63;
  final double BARGE_MAX = 7.41;

  public Command driveToBarge() {
    return Commands.defer(
        () -> {
          double y = swerveDrive.getEstimatedPose().getY();

          y = MathUtil.clamp(y, BARGE_MIN, BARGE_MAX);

          Pose2d pose = new Pose2d(BARGE_X, y, Rotation2d.fromDegrees(180));

          return swerveDrive.pathfindTo(pose).andThen(swerveDrive.alignTo(pose));
        },
        Set.of(swerveDrive.useMotion()));
  }

  public boolean hasCoral() {
    return false;
  }

  public Command autoOrientToAlgae() {
    return swerveDrive.facePoint(
        () ->
            Algae.getAlgaePosition(
                LIMELIGHT.ALGAE_CAMERA_NAME, swerveDrive, LIMELIGHT.ALGAE_CAMERA_POSITION));
  }

  public Command createDemoAutonomousCommand() {
    return Commands.sequence(
        placeCoral(new CoralPosition(0, 1)),
        pickupAlgae(3, 3),
        processAlgae(),
        placeCoral(new CoralPosition(11, 4)),
        pickupAlgae(5, 2),
        driveToBarge());
  }
}
