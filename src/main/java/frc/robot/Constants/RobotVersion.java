package frc.robot.Constants;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

public final class RobotVersion {
  public static enum Version {
    V1,
    V2
  }

  private static Version simVersion = Version.V2;

  public static Version get() {
    if (RobotBase.isSimulation()) return simVersion;

    String version = Preferences.getString("robot", null);

    if (version == null) {
      String chassis = Preferences.getString("Chassis", null);

      if (chassis == "COMPETITION") version = "v1";
      else if (chassis == "TEST") version = "v2";
      else version = "v2";
    }

    switch (version.toLowerCase()) {
      case "v1":
        return Version.V1;
      case "v2":
        return Version.V2;
      default:
        return Version.V1;
    }
  }

  public static boolean isV1() {
    return get() == Version.V1;
  }

  public static boolean isV2() {
    return get() == Version.V2;
  }

  private RobotVersion() {}
}
