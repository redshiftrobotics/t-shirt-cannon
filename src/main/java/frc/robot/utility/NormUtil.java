package frc.robot.utility;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Util to that measures the "length" or magnitude of a vector */
public class NormUtil {
  public static double norm(ChassisSpeeds speeds) {
    return norm(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public static double norm(double x, double y) {
    return Math.sqrt(x * x + y * y);
  }
}
