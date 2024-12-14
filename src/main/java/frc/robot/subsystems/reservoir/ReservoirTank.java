package frc.robot.subsystems.reservoir;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ThresholdController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem of t-shirt cannon robot representing the reservoir tank and its sensors and the
 * compressor which fills it.
 */
public class ReservoirTank extends SubsystemBase {

  private final ReservoirIO io;
  private final ReservoirIOInputsAutoLogged inputs = new ReservoirIOInputsAutoLogged();

  private final ThresholdController controller;

  private boolean paused;

  /** Create a new ReservoirTank subsystem */
  public ReservoirTank(ReservoirIO io) {
    this.io = io;

    paused = false;

    controller = new ThresholdController();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ReservoirTank", inputs);

    if (controller.calculate(inputs.tankPSI) > 0 && !paused && DriverStation.isEnabled()) {
      io.startCompressor();
    } else {
      io.stopCompressor();
    }
  }

  // --- Getters ---

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

  // --- Setters ---

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

  /** Pause the compressor */
  public void pause() {
    this.paused = true;
  }

  /** Unpause the compressor */
  public void unpause() {
    this.paused = false;
  }

  // --- Status String ---

  public String getStatusString() {
    if (isFilling()) {
      return String.format("Filling to %.2f PSI (End threshold)", controller.getUpperThreshold());
    } else if (!controller.isOn() && getPressure() > controller.getLowerThreshold()) {
      return String.format(
          "Stopped till %.2f PSI (Start threshold)", controller.getLowerThreshold());
    }
    if (this.paused) {
      return getCurrentCommand().getName();
    }
    return "Idle";
  }

  // --- Testing ---

  public void forceOpen() {
    System.out.println("Starting Compressor");
    io.startCompressor();
  }

  public void forceClose() {
    System.out.println("Stopping Compressor");
    io.stopCompressor();
  }
}
