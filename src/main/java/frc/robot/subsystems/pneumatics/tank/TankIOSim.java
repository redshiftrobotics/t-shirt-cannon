package frc.robot.subsystems.pneumatics.tank;

import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class TankIOSim implements TankIO {

  private final DoubleSupplier pressureFillSupplier;
  private final DoubleSupplier pressureDrainSupplier;

  private double tankPSI = 0;
  private boolean isFilling = false;

  /**
   * Create a new TankIOSim
   *
   * @param pressureFillSupplier a supplier of the pressure being added to the tank in psi per second
   * @param pressureDrainSupplier a supplier of the pressure being drained from the tank in psi per
   *     second
   */
  public TankIOSim(
      DoubleSupplier pressureFillSupplier, DoubleSupplier pressureDrainSupplier) {
    this.pressureFillSupplier = pressureFillSupplier;
    this.pressureDrainSupplier = pressureDrainSupplier;
  }

  @Override
  public void updateInputs(TankIOInputs inputs) {
    inputs.tankPSI = tankPSI;
    inputs.isFilling = isFilling;

    double pressureChangePerSecond = (inputs.isFilling ? pressureFillSupplier.getAsDouble() : 0) - pressureDrainSupplier.getAsDouble();

    tankPSI = Math.max(0, tankPSI + pressureChangePerSecond * Constants.LOOP_PERIOD_SECONDS);
  }

  @Override
  public void enable() {
    isFilling = true;
  }

  @Override
  public void disable() {
    isFilling = false;
  }
}
