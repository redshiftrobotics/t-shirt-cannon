package frc.robot.subsystems.pneumatics.gateway;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem of t-shirt cannon robot representing the gateway tank and the value which fills it,
 * along with the sensors on the gateway tank.
 */
public class GatewayTank extends SubsystemBase {

  private final GatewayIO io;
  private final GatewayIOInputsAutoLogged inputs = new GatewayIOInputsAutoLogged();

  private final InterpolatingShotTable shotTable;

  /** Create a new GatewayTank subsystem */
  public GatewayTank(GatewayIO io) {
    this.io = io;
    shotTable = new InterpolatingShotTable();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GatewayTank", inputs);
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
   * Get the estimated distance the t-shirt will travel based on the current pressure in the gateway
   *
   * @return distance in meters
   */
  public double getEstimatedLaunchDistance() {
    return shotTable.getEstimatedLaunchDistance(inputs.tankPSI);
  }

  /** Opens the value to fill the gateway tank. */
  public void startFilling() {
    io.openFillingValve();
  }

  /** Closes the valve. */
  public void stopFilling() {
    io.closeFillingValve();
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
