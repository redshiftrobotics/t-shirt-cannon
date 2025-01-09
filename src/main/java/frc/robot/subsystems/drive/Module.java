package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * An individual swerve module in a drivetrain. This class is above the IO layer and contains
 * functionality for using each module regardless of hardware specifics.
 */
public class Module {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final Translation2d distanceFromCenter;

  private SimpleMotorFeedforward driveFeedforward;
  private SwerveModuleState desiredState = new SwerveModuleState();

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnAbsoluteEncoderDisconnectedAlert;

  /**
   * Create an individual swerve module in a drivetrain.
   *
   * @param io swerve module io implantation
   * @param distanceFromCenter distance from center of drivetrain to physical center of swerve
   *     module
   */
  public Module(ModuleIO io, Translation2d distanceFromCenter) {
    this.io = io;
    this.distanceFromCenter = distanceFromCenter;

    driveFeedforward =
        new SimpleMotorFeedforward(
            DriveConstants.driveFeedforward.Ks(),
            DriveConstants.driveFeedforward.Kv(),
            DriveConstants.driveFeedforward.Ka(),
            Constants.LOOP_PERIOD_SECONDS);

    io.setDrivePID(DriveConstants.driveFeedback.Kp(), 0, DriveConstants.driveFeedback.Kd());
    io.setTurnPID(DriveConstants.turnFeedback.Kp(), 0, DriveConstants.turnFeedback.Kd());

    setBrakeMode(true);

    driveDisconnectedAlert =
        new Alert(
            String.format("Disconnected drive motor on module %s.", toString()), AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            String.format("Disconnected turn motor on module %s.", toString()), AlertType.kError);
    turnAbsoluteEncoderDisconnectedAlert =
        new Alert(
            String.format("Disconnected absolute turn encoder on module %s.", toString()),
            AlertType.kError);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    Logger.processInputs("Drive/" + toString(), inputs);
    io.updateInputs(inputs);

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveMotorConnected);
    turnDisconnectedAlert.set(!inputs.turnMotorConnected);
    turnAbsoluteEncoderDisconnectedAlert.set(!inputs.turnAbsoluteEncoderConnected);
  }

  // --- Odometry ---

  /** Get all latest {@link SwerveModulePosition}'s from last cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    SwerveModulePosition[] odometryPositions = new SwerveModulePosition[sampleCount];

    for (int i = 0; i < sampleCount; i++) {

      double positionMeters = inputs.odometryDrivePositionsRad[i] * DRIVE_CONFIG.wheelRadius();
      Rotation2d angle = inputs.odometryTurnPositions[i];

      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  // --- Speeds ---

  /** Runs the module with the specified setpoint state. */
  public void setSpeeds(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null

    state.optimize(getAngle());
    state.cosineScale(getAngle());

    double velocityRadiansPerSecond = state.speedMetersPerSecond / DRIVE_CONFIG.wheelRadius();
    double angleRadians = state.angle.getRadians();

    io.setDriveVelocity(
        velocityRadiansPerSecond, driveFeedforward.calculate(state.speedMetersPerSecond));
    io.setTurnPosition(angleRadians);

    desiredState = state;
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getSpeeds() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the setpoint module state (turn angle and drive velocity) */
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  // --- Position ---

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  // --- Characterization ---

  /** Runs characterization volts at voltage. */
  public void runCharacterization(double turnSetpointRads, double volts) {
    io.setTurnPosition(turnSetpointRads);
    io.setDriveVoltage(volts);
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  // --- Kinematics ---

  /** Returns the distance module is from center of robot */
  public Translation2d getDistanceFromCenter() {
    return distanceFromCenter;
  }

  // --- Mode Requests ---

  /** Disables all outputs to motors. */
  public void stop() {
    io.stop();
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  // --- Position and Speed Component Getters ---

  /** Returns the current turn angle of the module. */
  private Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Returns the current drive position of the module in meters. */
  private double getPositionMeters() {
    return inputs.drivePositionRad * DRIVE_CONFIG.wheelRadius();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  private double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DRIVE_CONFIG.wheelRadius();
  }

  // --- To String ---

  @Override
  public String toString() {

    final String[] yPositions = {"Back", "Middle", "Front"};
    final String[] xPositions = {"Right", "Middle", "Left"};

    final int ySignum = (int) Math.signum(distanceFromCenter.getY());
    final int xSignum = (int) Math.signum(distanceFromCenter.getX());

    return xPositions[xSignum + 1] + yPositions[ySignum + 1] + "SwerveModule";
  }
}
