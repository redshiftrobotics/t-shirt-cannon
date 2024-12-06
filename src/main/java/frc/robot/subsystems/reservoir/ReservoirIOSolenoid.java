package frc.robot.subsystems.reservoir;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.hardwareWrappers.CustomPneumaticHub;

public class ReservoirIOSolenoid implements ReservoirIO {

  private CustomPneumaticHub hub;
  private Solenoid solenoid;

  public ReservoirIOSolenoid() {
    hub = new CustomPneumaticHub();
    solenoid = hub.makeSolenoid(8);
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = hub.getPressure(0);
    inputs.compressorRunning = solenoid.get();
  }

  @Override
  public void startCompressor() {
    solenoid.set(true);
  }

  @Override
  public void stopCompressor() {
    solenoid.set(false);
  }
}
