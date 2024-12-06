package frc.robot.subsystems.reservoir;

import frc.robot.Constants;

public class ReservoirIOSim implements ReservoirIO {

  private static final double PSI_CHANGE_PER_SECOND = 1.2;

  private double tankPSI;
  private boolean compressorRunning;

  public ReservoirIOSim() {
    tankPSI = ReservoirConstants.AVERAGE_ATMOSPHERIC_PSI;
    compressorRunning = false;
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = tankPSI;
    inputs.compressorRunning = compressorRunning;

    tankPSI += inputs.compressorRunning ? PSI_CHANGE_PER_SECOND * Constants.LOOP_PERIOD_SECONDS : 0;
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
