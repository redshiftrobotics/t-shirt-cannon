package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class SwerveJoystickUtil {

  public static final double JOYSTICK_DEADBAND = 0.15;
  public static final double ANGLE_DEADBAND = 0.5;

  public static final double LINEAR_VELOCITY_EXPONENT = 2.0; // Square the joystick input
  public static final double ANGULAR_VELOCITY_EXPONENT = 2.0; // Square the joystick input

  public static Translation2d getTranslationMetersPerSecond(
      double xInput, double yInput, double maxTranslationSpeedMetersPerSecond) {

    if (!DriverStation.isTeleopEnabled()) return Translation2d.kZero;

    Translation2d translation = new Translation2d(xInput, yInput);

    // get length of linear velocity vector, and apply deadband to it for noise reduction
    double magnitude = MathUtil.applyDeadband(translation.getNorm(), JOYSTICK_DEADBAND);

    if (magnitude == 0) return Translation2d.kZero;

    // squaring the magnitude of the vector makes for quicker ramp up and slower fine control,
    // magnitude should always be positive
    double magnitudeSquared = Math.abs(Math.pow(magnitude, LINEAR_VELOCITY_EXPONENT));

    // get a vector with the same angle as the base linear velocity vector but with the
    // magnitude squared
    Translation2d squaredLinearVelocity =
        new Translation2d(magnitudeSquared, translation.getAngle());

    // return final value
    return squaredLinearVelocity.times(maxTranslationSpeedMetersPerSecond);
  }

  public static double getOmegaRadiansPerSecond(
      double omegaInput, double maxAngularSpeedRadPerSec) {

    if (!DriverStation.isTeleopEnabled()) return 0.0;

    // get rotation speed, and apply deadband
    double omega = MathUtil.applyDeadband(omegaInput, JOYSTICK_DEADBAND);

    // square the omega value for quicker ramp up and slower fine control
    // make sure to copy the sign over for direction
    double omegaSquared = Math.copySign(Math.pow(omega, ANGULAR_VELOCITY_EXPONENT), omega);

    // return final value
    return omegaSquared * maxAngularSpeedRadPerSec;
  }

  public static Optional<Rotation2d> getHeadingDirection(double xInput, double yInput) {

    if (!DriverStation.isTeleopEnabled()) return Optional.empty();

    // Get desired angle as a vector
    final Translation2d desiredAngle = new Translation2d(xInput, yInput);

    // If the vector length is less than the deadband, return null
    if (desiredAngle.getNorm() < ANGLE_DEADBAND) {
      return Optional.empty();
    }

    return Optional.of(desiredAngle.getAngle());
  }
}
