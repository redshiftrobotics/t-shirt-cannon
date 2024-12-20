package frc.robot.subsystems.pneumatics.tank;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.pneumatics.hardwareWrappers.Transducer;

public class TankIOSolenoid implements TankIO {

  private final Solenoid solenoid;
  private final Transducer pressureSensor;

  public TankIOSolenoid() {
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);
    pressureSensor = new Transducer(1);
  }

  @Override
  public void updateInputs(TankIOInputs inputs) {
    inputs.isFilling = solenoid.get();
    inputs.tankPSI = pressureSensor.getTankPSI();
  }

  @Override
  public void enable() {
    solenoid.set(true);
  }

  @Override
  public void disable() {
    solenoid.set(false);
  }
}
