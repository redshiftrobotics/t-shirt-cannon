package frc.robot.subsystems.pneumatics.reservoir;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.subsystems.pneumatics.hardwareWrappers.Transducer;

public class ReservoirIOHardware implements ReservoirIO {

  private final Transducer pressureSensor;
  private final Relay output;

  public ReservoirIOHardware() {
    output = new Relay(0, Direction.kBoth);
    pressureSensor = new Transducer(0);
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = pressureSensor.getTankPSI();
    inputs.compressorRunning = output.get().equals(Value.kOn);
  }

  @Override
  public void startCompressor() {
    // SmartDashboard.putBoolean("Reservoir", true);
    output.set(Value.kOn);
  }

  @Override
  public void stopCompressor() {
    // SmartDashboard.putBoolean("Reservoir", false);
    output.set(Value.kOff);
  }
}
