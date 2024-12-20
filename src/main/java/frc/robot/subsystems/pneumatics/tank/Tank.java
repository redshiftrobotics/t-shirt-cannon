package frc.robot.subsystems.pneumatics.tank;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem of t-shirt cannon robot representing the reservoir tank and its sensors and the
 * compressor which fills it.
 */
public class Tank extends SubsystemBase {

  private final TankIO io;
  private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();

  /** Create a new ReservoirTank subsystem */
  public Tank(TankIO io) {
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
    return inputs.isFilling;
  }

  /**
   * Get current pressure of reservoir tank
   *
   * @return pressure in psi (pound per square inch)
   */
  public double getPressure() {
    return inputs.tankPSI;
  }

  /** Enables the system to start filling tank */
  private void enable() {
    io.enable();
  }

  /** Disables the system to stop filling tank */
  private void disable() {
    io.disable();
  }

  /** Checks if the pressure is below mas pressure*/
  private boolean isSafePressure() {
    return getPressure() < TankConstants.MAX_ALLOWED_PRESSURE;
  }

  /**
   * Fills the tank until the pressure is greater than or equal to the given pressure
   *
   * @param pressure the pressure to fill the tank to
   * @return the command to fill the tank to the given pressure
   */
  public Command fillToPressure(double pressure) {
    return fill()
        .until(() -> getPressure() >= pressure)
        .andThen(pauseFill())
        .withName(String.format("Filling to %s PSI", pressure));
  }

  public Command pauseTillPressure(double pressure) {
    return pauseFill()
        .until(() -> getPressure() <= pressure)
        .withName(String.format("Pausing till %s PSI", pressure));
  }

  public Command fill() {
    return this.startEnd(this::enable, this::disable)
        .onlyIf(this::isSafePressure)
        .andThen(Commands.idle(this))
        .withName("Filling Tank");
  }

  public Command pauseFill() {
    return this.runOnce(this::disable)
      .andThen(Commands.idle(this))
      .withName("Pause Filling Tank");
  }
}
