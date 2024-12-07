package frc.robot.subsystems.cannon;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.hardwareWrappers.CustomPneumaticHub;

public class CannonIOHardware implements CannonIO {

  private final Solenoid solenoid;

  public CannonIOHardware(int solenoidChannel) {
    try (CustomPneumaticHub hub = new CustomPneumaticHub()) {
      solenoid = hub.makeSolenoid(solenoidChannel);
    }
  }

  @Override
  public void updateInputs(CannonIOInputs inputs) {
    inputs.isOpen = solenoid.get();
  }

  @Override
  public void open() {
    solenoid.set(true);
  }

  @Override
  public void close() {
    solenoid.set(false);
  }
}
