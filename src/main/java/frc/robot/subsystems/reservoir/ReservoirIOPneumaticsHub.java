package frc.robot.subsystems.reservoir;

import frc.robot.hardwareWrappers.CustomPneumaticHub;

public class ReservoirIOPneumaticsHub implements ReservoirIO {

  private final CustomPneumaticHub hub;

  public ReservoirIOPneumaticsHub() {
    hub = new CustomPneumaticHub();
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.compressorRunning = hub.getCompressor();
    inputs.tankPSI = hub.getPressure(0);
  }

  /** Enables the closed loop control of the compressor. */
  @Override
  public void setTargetPressure(double pressure) {
    hub.enableCompressorAnalog(Math.max(pressure - ReservoirConstants.PSI_TOLERANCE, 0), pressure);
  }
}
