package frc.robot.util.software;

import edu.wpi.first.math.geometry.Translation2d;

/** Some math utility functions for the swerve drive and input processing. */
public final class MathUtils {
  /** Math for taking input from the controller. */
  public static final class InputMath {
    /** Deadband must be greater than 1e-6 */
    public static Translation2d addCircularDeadband(Translation2d input, double deadband) {
      double magnitude = input.getNorm();

      if (Math.abs(magnitude) <= deadband) return new Translation2d();

      double mappedMagnitude = nonLinear(map(magnitude, deadband, 1.0, 0.0, 1.0));

      return input.times(mappedMagnitude / magnitude);
    }

    public static double nonLinear(double x) {
      return 1 - Math.cos(x * Math.PI / 2.0);
    }
  }

  /**
   * Maps a value from one range to another.
   *
   * @param X The value to map.
   * @param A The lower bound of the value's current range.
   * @param B The upper bound of the value's current range.
   * @param C The lower bound of the value's target range.
   * @param D The upper bound of the value's target range.
   * @return X, mapped from [A, B] to [C, D].
   */
  public static double map(double X, double A, double B, double C, double D) {
    return (X - A) / (B - A) * (D - C) + C;
  }

  /**
   * Computes the modulus of a number rounding down instead of towards 0. Also equivalent to x -
   * floor(x / r) * r.
   *
   * @param x The number to mod.
   * @param r The modulus.
   * @return x mod r, rounded down.
   */
  public static double floorMod(double x, double r) {
    return ((x % r) + r) % r;
  }

  public static boolean isInsideTriangle(
      Translation2d A, Translation2d B, Translation2d C, Translation2d P) {
    // Calculate the barycentric coordinates
    // of point P with respect to triangle ABC
    double denominator =
        ((B.getY() - C.getY()) * (A.getX() - C.getX())
            + (C.getX() - B.getX()) * (A.getY() - C.getY()));
    double a =
        ((B.getY() - C.getY()) * (P.getX() - C.getX())
                + (C.getX() - B.getX()) * (P.getY() - C.getY()))
            / denominator;
    double b =
        ((C.getY() - A.getY()) * (P.getX() - C.getX())
                + (A.getX() - C.getX()) * (P.getY() - C.getY()))
            / denominator;
    double c = 1 - a - b;

    // Check if all barycentric coordinates
    // are non-negative
    if (a >= 0 && b >= 0 && c >= 0) {
      return true;
    } else {
      return false;
    }
  }
}
