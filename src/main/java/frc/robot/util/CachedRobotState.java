package frc.robot.util;

import com.team6962.lib.telemetry.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class CachedRobotState extends SubsystemBase {
  private static CachedRobotState instance;

  public static CachedRobotState getInstance() {
    if (instance == null) {
      instance = new CachedRobotState();
    }
    return instance;
  }

  private CachedRobotState() {
    Logger.logBoolean("RobotState/isAutonomous", CachedRobotState::isAutonomous);
    Logger.logBoolean("RobotState/isDisabled", CachedRobotState::isDisabled);
    Logger.logBoolean("RobotState/isTeleop", CachedRobotState::isTeleop);
    Logger.logBoolean("RobotState/isTest", CachedRobotState::isTest);
    Logger.logBoolean("RobotState/isEStopped", CachedRobotState::isEStopped);
    Logger.logBoolean("RobotState/isEnabled", CachedRobotState::isEnabled);
    Logger.logBoolean("RobotState/isFMSAttached", CachedRobotState::isFMSAttached);
    Logger.logBoolean("RobotState/isDSAttached", CachedRobotState::isDSAttached);
    Logger.logBoolean("RobotState/knowsAlliance", CachedRobotState::knowsAlliance);
    Logger.logString(
        "RobotState/alliance", () -> alliance.map(a -> a.toString()).orElse("Unknown"));
  }

  private static boolean isAutonomous;
  private static boolean isDisabled;
  private static boolean isTeleop;
  private static boolean isTest;
  private static boolean isEStopped;
  private static boolean isEnabled;
  private static boolean isFMSAttached;
  private static boolean isDSAttached;

  private static Optional<Alliance> alliance = Optional.empty();

  @Override
  public void periodic() {
    isAutonomous = RobotState.isAutonomous();
    isDisabled = RobotState.isDisabled();
    isTeleop = RobotState.isTeleop();
    isTest = RobotState.isTest();
    isEStopped = RobotState.isEStopped();
    isEnabled = RobotState.isEnabled();

    isFMSAttached = DriverStation.isFMSAttached();
    isDSAttached = DriverStation.isDSAttached();

    alliance =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Optional.of(Alliance.Red)
            : Optional.of(Alliance.Blue);

    Logger.log("ImmediateRobotState/isAutonomous", isAutonomous);
    Logger.log("ImmediateRobotState/isDisabled", isDisabled);
    Logger.log("ImmediateRobotState/isTeleop", isTeleop);
    Logger.log("ImmediateRobotState/isTest", isTest);
    Logger.log("ImmediateRobotState/isEStopped", isEStopped);
    Logger.log("ImmediateRobotState/isEnabled", isEnabled);
    Logger.log("ImmediateRobotState/isFMSAttached", isFMSAttached);
    Logger.log("ImmediateRobotState/isDSAttached", isDSAttached);
    Logger.log("ImmediateRobotState/alliance", alliance.map(a -> a.toString()).orElse("Unknown"));
    Logger.log("ImmediateRobotState/lastCache", Timer.getFPGATimestamp());
  }

  public static boolean isAutonomous() {
    return isAutonomous;
  }

  public static boolean isDisabled() {
    return isDisabled;
  }

  public static boolean isTeleop() {
    return isTeleop;
  }

  public static boolean isTest() {
    return isTest;
  }

  public static boolean isEStopped() {
    return isEStopped;
  }

  public static boolean isEnabled() {
    return isEnabled;
  }

  public static boolean isFMSAttached() {
    return isFMSAttached;
  }

  public static boolean isDSAttached() {
    return isDSAttached;
  }

  public static Alliance getAlliance() {
    return alliance.orElse(Alliance.Blue);
  }

  public static Optional<Boolean> isAllianceInverted() {
    return isRed();
  }

  public static Optional<Boolean> isBlue() {
    return alliance.map(a -> a == Alliance.Blue);
  }

  public static Optional<Boolean> isRed() {
    return alliance.map(a -> a == Alliance.Red);
  }

  public static boolean knowsAlliance() {
    return alliance.isPresent();
  }

  public static void init() {
    getInstance();
  }
}
