package frc.robot.subsystems.gateway;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class GatewayIOHardware implements GatewayIO {

  private final Solenoid solenoid;

  public GatewayIOHardware() {
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);
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
