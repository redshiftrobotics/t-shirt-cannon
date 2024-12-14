package frc.robot.subsystems.gateway;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.hardwareWrappers.Transducer;

public class GatewayIOHardware implements GatewayIO {

  private final Solenoid solenoid;
  private final Transducer pressureSensor;

  public GatewayIOHardware() {
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);
    pressureSensor = new Transducer(1);
  }

  @Override
  public void updateInputs(GatewayIOInputs inputs) {
    inputs.isFilling = solenoid.get();
    inputs.tankPSI = pressureSensor.getTankPSI();
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
