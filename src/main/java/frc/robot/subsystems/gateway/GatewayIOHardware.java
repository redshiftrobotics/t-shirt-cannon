package frc.robot.subsystems.gateway;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putBoolean("Gateway", true);
    solenoid.set(true);
  }

  @Override
  public void stopFilling() {
    SmartDashboard.putBoolean("Gateway", false);
    solenoid.set(false);
  }
}
