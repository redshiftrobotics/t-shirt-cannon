package frc.robot.subsystems.gateway;

import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class GatewayIOSim implements GatewayIO {

  // Completely made up values, just to simulate the system
  private static final double FILLING_PSI_CHANGE_PER_SECOND = 5;
  private static final double DRAIN_PSI_CHANGE_PER_SECOND = 20;
  private static final double PASSIVE_PSI_LEAK_PER_SECOND = 0.02;

  private BooleanSupplier isDrainingSupplier;

  private double tankPSI;
  private boolean isFilling;

  public GatewayIOSim() {
    tankPSI = 0;
    isFilling = false;
  }

  @Override
  public void updateInputs(GatewayIOInputs inputs) {
    inputs.tankPSI = tankPSI;
    inputs.isFilling = isFilling;

    double pressureChangePerSecond =
        0
            + (isFilling ? FILLING_PSI_CHANGE_PER_SECOND : 0)
            - (isDrainingSupplier.getAsBoolean() ? DRAIN_PSI_CHANGE_PER_SECOND : 0)
            - PASSIVE_PSI_LEAK_PER_SECOND;

    tankPSI = Math.max(0, tankPSI + pressureChangePerSecond * Constants.LOOP_PERIOD_SECONDS);
  }

  @Override
  public void beganFilling() {
    isFilling = true;
  }

  @Override
  public void stopFilling() {
    isFilling = false;
  }

  @Override
  public void setSimDrain(BooleanSupplier isDrainingSupplier) {
    this.isDrainingSupplier = isDrainingSupplier;
  }
}
