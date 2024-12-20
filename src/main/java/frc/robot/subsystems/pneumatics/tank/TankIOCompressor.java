package frc.robot.subsystems.pneumatics.tank;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.subsystems.pneumatics.hardwareWrappers.Transducer;

public class TankIOCompressor implements TankIO {

  private final Transducer pressureSensor;
  private final Relay output;

  public TankIOCompressor() {
    output = new Relay(0, Direction.kBoth);
    pressureSensor = new Transducer(0);
  }

  @Override
  public void updateInputs(TankIOInputs inputs) {
    inputs.tankPSI = pressureSensor.getTankPSI();
    inputs.isFilling = output.get().equals(Value.kOn);
  }

  @Override
  public void enable() {
    output.set(Value.kOn);
  }

  @Override
  public void disable() {
    output.set(Value.kOff);
  }
}
