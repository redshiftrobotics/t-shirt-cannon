package frc.robot.subsystems.reservoir;

import frc.robot.Constants;

public class ReservoirIOSim implements ReservoirIO {

  private static final double COMPRESSOR_PSI_CHANGE_PER_SECOND = 1.25;
  private static final double PASSIVE_PSI_LEAK_PER_SECOND = 0.02;

  private double tankPSI;
  private boolean compressorRunning;

  public ReservoirIOSim() {
    tankPSI = 0;
    compressorRunning = false;
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = tankPSI;
    inputs.compressorRunning = compressorRunning;

    if (inputs.compressorRunning) {
      tankPSI += COMPRESSOR_PSI_CHANGE_PER_SECOND * Constants.LOOP_PERIOD_SECONDS;
    }
    tankPSI = Math.max(tankPSI - PASSIVE_PSI_LEAK_PER_SECOND * Constants.LOOP_PERIOD_SECONDS, 0);
  }

  @Override
  public void startCompressor() {
    compressorRunning = true;
  }

  @Override
  public void stopCompressor() {
    compressorRunning = false;
  }
}
