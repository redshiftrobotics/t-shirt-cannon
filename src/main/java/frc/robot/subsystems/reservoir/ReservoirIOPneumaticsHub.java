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
    // SmartDashboard.putNumber("Analog V 0", hub.getAnalogVoltage(0));
    // SmartDashboard.putNumber("Analog V 1", hub.getAnalogVoltage(1));
    // SmartDashboard.putNumber("PSI V 0", hub.getPressure(0));
    // SmartDashboard.putNumber("PSI V 1", hub.getPressure(1));
  }

  /** Enables the closed loop control of the compressor. */
  @Override
  public void setTargetPressure(double pressure) {
    // SmartDashboard.putNumber(
    //     "Min Target", Math.max(pressure - ReservoirConstants.PSI_TOLERANCE, 0));
    // SmartDashboard.putNumber("Max Target", pressure);
    hub.enableCompressorAnalog(Math.max(pressure - ReservoirConstants.PSI_TOLERANCE, 0), pressure);
  } 
}
