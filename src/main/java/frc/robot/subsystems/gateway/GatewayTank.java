package frc.robot.subsystems.gateway;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ThresholdController;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem of t-shirt cannon robot representing the gateway tank and the value which fills it,
 * along with the sensors on the gateway tank.
 */
public class GatewayTank extends SubsystemBase {

  private final GatewayIO io;
  private final GatewayIOInputsAutoLogged inputs = new GatewayIOInputsAutoLogged();

  private final ThresholdController controller;

  private final InterpolatingShotTable shotTable;

  private final ArrayList<PauseCondition> pauseConditions;

  private double targetShotDistance;

  /** Create a new GatewayTank subsystem */
  public GatewayTank(GatewayIO io) {
    this.io = io;

    pauseConditions = new ArrayList<>();

    controller = new ThresholdController();

    shotTable = new InterpolatingShotTable();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GatewayTank", inputs);

    if (controller.calculate(inputs.tankPSI) > 0
        && !shouldPauseFilling()
        && DriverStation.isEnabled()) {
      io.beganFilling();
    } else {
      io.stopFilling();
    }

    Logger.recordOutput("GatewayTank/status", getStatusString());
  }

  // --- Getters ---

  /**
   * Get whether compressor is currently active and filling Gateway tank
   *
   * @return compressor state
   */
  public boolean isFilling() {
    return inputs.isFilling;
  }

  /**
   * Get current pressure of gateway tank
   *
   * @return pressure in psi (pound per square inch)
   */
  public double getPressure() {
    return inputs.tankPSI;
  }

  /**
   * Get whether pressure is within the tolerance range
   *
   * @return true if pressure is within tolerance range, false otherwise
   */
  public boolean isPressureWithinTolerance() {
    return MathUtil.isNear(
        getPressure(),
        shotTable.getDesiredPSI(targetShotDistance),
        GatewayConstants.TOLERANCE_PRESSURE);
  }

  /**
   * Get the estimated distance the t-shirt will travel based on the current pressure in the gateway
   *
   * @return distance in meters
   */
  public double getEstimatedLaunchDistance() {
    return shotTable.getEstimatedLaunchDistance(inputs.tankPSI);
  }

  /**
   * Get the target distance the t-shirt will travel
   *
   * @return distance in meters
   */
  public double getTargetLaunchDistance() {
    return targetShotDistance;
  }

  // --- Setters ---

  /**
   * Set the target distance the t-shirt will travel, changes target pressure of the gateway tank
   *
   * @param distance in meters
   */
  public void setTargetLaunchDistance(double distance) {
    distance =
        MathUtil.clamp(
            distance, GatewayConstants.MIN_SHOT_DISTANCE, GatewayConstants.MAX_SHOT_DISTANCE);
    setDesiredPSI(shotTable.getDesiredPSI(distance));
  }

  /**
   * Set the target pressure of the gateway tank
   *
   * @param psi target pressure in psi (pound per square inch)
   */
  public void setDesiredPSI(double psi) {
    double min = GatewayConstants.MIN_ALLOWED_PRESSURE;
    double max = GatewayConstants.MAX_ALLOWED_PRESSURE;
    double tolerance = GatewayConstants.TOLERANCE_PRESSURE;

    psi = MathUtil.clamp(psi, min, max);

    controller.setThresholds(Math.max(psi - tolerance, min), psi);

    targetShotDistance = shotTable.getEstimatedLaunchDistance(psi);
  }

  /** Sets the setpoint pressure to none. The compressor will not activate. */
  public void stopFilling() {
    controller.setThresholds(0, 0);
  }

  // --- Sim Drain ---

  /**
   * Sets a supplier that tells the sim whether it is draining.
   *
   * @param isDrainingSupplier A supplier that returns true if the tank is draining air.
   */
  public void setSimDrain(BooleanSupplier isDrainingSupplier) {
    this.io.setSimDrain(isDrainingSupplier);
  }

  // --- Pause Conditions ---

  public void addPauseFillingCondition(BooleanSupplier condition, String reason) {
    pauseConditions.add(new PauseCondition(condition, reason));
  }

  public void addPauseFillingCondition(
      BooleanSupplier condition, String reason, double debounceTimeSeconds) {
    final Debouncer debouncer = new Debouncer(debounceTimeSeconds, DebounceType.kBoth);
    pauseConditions.add(
        new PauseCondition(() -> debouncer.calculate(condition.getAsBoolean()), reason));
  }

  public boolean shouldPauseFilling() {
    return pauseConditions.stream().anyMatch(PauseCondition::isActive);
  }

  public Optional<String> getPauseReason() {
    return pauseConditions.stream()
        .filter(PauseCondition::isActive)
        .map(PauseCondition::reason)
        .findFirst();
  }

  public String getStatusString() {
    if (isFilling()) {
      return String.format("Filling to %.2f PSI (End threshold)", controller.getUpperThreshold());
    } else if (!controller.isOn() && getPressure() > controller.getLowerThreshold()) {
      return String.format(
          "Stopped till %.2f PSI (Start threshold)", controller.getLowerThreshold());
    }
    if (shouldPauseFilling()) {
      return "Paused: " + getPauseReason().orElse("Unknown");
    }
    return "Idle";
  }

  public static record PauseCondition(BooleanSupplier condition, String reason) {
    public boolean isActive() {
      return condition.getAsBoolean();
    }
  }
}
