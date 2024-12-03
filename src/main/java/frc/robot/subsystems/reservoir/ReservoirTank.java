package frc.robot.subsystems.reservoir;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem of t-shirt cannon robot representing the reservoir tank and the compressor which fills
 * it, along with the sensors on the reservoir tank.
 */
public class ReservoirTank extends SubsystemBase {

  private final ReservoirIO io;
  private final ReservoirIOInputsAutoLogged inputs = new ReservoirIOInputsAutoLogged();

  public ReservoirTank(ReservoirIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ReservoirTank", inputs);
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
   * @return pressure in pound per square inch (PSI)
   */
  public double getPressure() {
    return inputs.tankPSI;
  }

  /** Sets the setpoint pressure to full. The compressor will try and maintain this pressure. */
  public void setDesiredPressureToFull() {
    setDesiredPressure(ReservoirConstants.FULL_TANK_PSI);
  }

  /** Sets the setpoint pressure to none. The compressor will not activate. */
  public void stopFilling() {
    setDesiredPressure(0);
  }

  /**
   * Sets the setpoint pressure that the compressor will try and bring the reservoir tank to. Units
   * are PSI (or more accurately lbf/in^2).
   *
   * @param psi setpoint pressure in pound per square inch (PSI)
   * @throws IllegalArgumentException if desired pressure is set to negative psi
   */
  public void setDesiredPressure(double psi) {
    if (psi < 0) {
      throw new IllegalArgumentException("Unable to set desired pressure to negative PSI");
    }
    if (psi > ReservoirConstants.FULL_TANK_PSI) {
      throw new IllegalArgumentException("Unable to set desired pressure to above max PSI");
    }

    io.setTargetPressure(psi);
  }
}
