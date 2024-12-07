package frc.robot.subsystems.reservoir;

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
 * Subsystem of t-shirt cannon robot representing the reservoir tank and its sensors and the compressor which fills
 * it.
 */
public class ReservoirTank extends SubsystemBase {

  private final ReservoirIO io;
  private final ReservoirIOInputsAutoLogged inputs = new ReservoirIOInputsAutoLogged();

  private final ThresholdController controller;

  private final ArrayList<PauseCondition> pauseConditions;

  /** Create a new ReservoirTank subsystem */
  public ReservoirTank(ReservoirIO io) {
    this.io = io;

    pauseConditions = new ArrayList<>();

    controller = new ThresholdController();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ReservoirTank", inputs);

    if (controller.calculate(inputs.tankPSI) > 0
        && !shouldPauseFilling()
        && DriverStation.isEnabled()) {
      io.startCompressor();
    } else {
      io.stopCompressor();
    }

    Logger.recordOutput("ReservoirTank/status", getStatusString());
  }

  /**
   * Get whether compressor is currently active and filling reservoir tank
   *
   * @return compressor state
   */
  public boolean isFilling() {
    return inputs.compressorRunning;
  }

  /**
   * Get current pressure of reservoir tank
   *
   * @return pressure in psi (pound per square inch)
   */
  public double getPressure() {
    return inputs.tankPSI;
  }

  /** Sets the setpoint pressure to none. The compressor will not activate. */
  public void stopFilling() {
    controller.setThresholds(0, 0);
  }

  /**
   * Enables the compressor to fill the reservoir tank to the specified pressure.
   *
   * @param minPressure The minimum pressure in PSI. The compressor will turn on when the pressure
   *     drops below this value and the pressure switch indicates that the system is not full. Range
   *     0-120 PSI.
   * @param maxPressure The maximum pressure in PSI. The compressor will turn off when the pressure
   *     reaches this value or the pressure switch is disconnected or indicates that the system is
   *     full. Range 0-120 PSI. Must be larger then minPressure.
   */
  public void setPressureThresholds(double minPressure, double maxPressure) {
    if (minPressure >= maxPressure) {
      throw new IllegalArgumentException("maxPressure must be greater than minPressure");
    }
    if (minPressure < ReservoirConstants.MIN_ALLOWED_PRESSURE
        || minPressure > ReservoirConstants.MAX_ALLOWED_PRESSURE) {
      throw new IllegalArgumentException(
          String.format(
              "minPressure must be between %s and %s PSI, got %s",
              ReservoirConstants.MIN_ALLOWED_PRESSURE,
              ReservoirConstants.MAX_ALLOWED_PRESSURE,
              minPressure));
    }
    if (maxPressure < ReservoirConstants.MIN_ALLOWED_PRESSURE
        || maxPressure > ReservoirConstants.MAX_ALLOWED_PRESSURE) {
      throw new IllegalArgumentException(
          String.format(
              "maxPressure must be between %s and %s PSI, got %s",
              ReservoirConstants.MIN_ALLOWED_PRESSURE,
              ReservoirConstants.MAX_ALLOWED_PRESSURE,
              maxPressure));
    }

    controller.setThresholds(minPressure, maxPressure);
  }

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
      return String.format("Filling to %.2f PSI", controller.getUpperThreshold());
    } else if (!controller.isOn()) {
      return String.format(
          "Waiting to fill till pressure below %.2f PSI", controller.getLowerThreshold());
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
