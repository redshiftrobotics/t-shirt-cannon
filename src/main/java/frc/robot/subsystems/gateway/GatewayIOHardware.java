package frc.robot.subsystems.gateway;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.hardwareWrappers.CustomPneumaticHub;

public class GatewayIOHardware implements GatewayIO {

  private final Solenoid solenoid;

  public GatewayIOHardware() {
    try (CustomPneumaticHub hub = new CustomPneumaticHub()) {
      solenoid = hub.makeSolenoid(15);
    }
  }

  @Override
  public void updateInputs(GatewayIOInputs inputs) {
    inputs.isFilling = solenoid.get();
    inputs.tankPSI = 0;
  }

  @Override
  public void beganFilling() {
    solenoid.set(true);
  }

  @Override
  public void stopFilling() {
    solenoid.set(false);
  }
}
