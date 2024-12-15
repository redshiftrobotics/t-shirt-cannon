package frc.robot.subsystems.pneumatics.reservoir;

import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class ReservoirIOSim implements ReservoirIO {

  // Completely made up values, just to simulate the system
  private static final double COMPRESSOR_PSI_CHANGE_PER_SECOND = 1.25;
  private static final double DRAIN_PSI_CHANGE_PER_SECOND = 3;
  private static final double PASSIVE_PSI_LEAK_PER_SECOND = 0.02;

  private BooleanSupplier isDrainingSupplier;

  private double tankPSI;
  private boolean compressorRunning;

  public ReservoirIOSim() {
    tankPSI = 0;
    compressorRunning = false;
    isDrainingSupplier = () -> false;
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = tankPSI;
    inputs.compressorRunning = compressorRunning;

    double pressureChangePerSecond =
        0
            + (compressorRunning ? COMPRESSOR_PSI_CHANGE_PER_SECOND : 0)
            - (isDrainingSupplier.getAsBoolean() ? DRAIN_PSI_CHANGE_PER_SECOND : 0)
            - PASSIVE_PSI_LEAK_PER_SECOND;

    tankPSI = Math.max(0, tankPSI + pressureChangePerSecond * Constants.LOOP_PERIOD_SECONDS);
  }

  @Override
  public void startCompressor() {
    compressorRunning = true;
  }

  @Override
  public void stopCompressor() {
    compressorRunning = false;
  }

  @Override
  public void setSimDrain(BooleanSupplier isDrainingSupplier) {
    this.isDrainingSupplier = isDrainingSupplier;
  }
}
