package frc.robot.subsystems.reservoir;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.hardwareWrappers.CustomPneumaticHub;

public class ReservoirIODigital implements ReservoirIO {

  private CustomPneumaticHub hub;
  private Relay output;

  public ReservoirIODigital() {
    hub = new CustomPneumaticHub();
    output = new Relay(0, Direction.kBoth);
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = hub.getPressure(1);
    inputs.compressorRunning = output.get().equals(Value.kOn);
  }

  @Override
  public void startCompressor() {
    System.out.println("Starting compressor");
    output.set(Value.kOn);
  }

  @Override
  public void stopCompressor() {
    System.out.println("Stopping compressor");
    output.set(Value.kOff);
  }
}
