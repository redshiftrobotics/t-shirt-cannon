package frc.robot.subsystems.pneumatics.reservoir;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem of t-shirt cannon robot representing the reservoir tank and its sensors and the
 * compressor which fills it.
 */
public class ReservoirTank extends SubsystemBase {

  private final ReservoirIO io;
  private final ReservoirIOInputsAutoLogged inputs = new ReservoirIOInputsAutoLogged();

  /** Create a new ReservoirTank subsystem */
  public ReservoirTank(ReservoirIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ReservoirTank", inputs);
  }

  /** Enables the compressor to fill the reservoir tank. */
  public void startFilling() {
    io.startCompressor();
  }

  /** Disables the compressor. */
  public void stopFilling() {
    io.stopCompressor();
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

  /**
   * Sets a supplier that tells the sim whether it is draining.
   *
   * @param isDrainingSupplier A supplier that returns true if the tank is draining air.
   */
  public void setSimDrain(BooleanSupplier isDrainingSupplier) {
    this.io.setSimDrain(isDrainingSupplier);
  }
}
