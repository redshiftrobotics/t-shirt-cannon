package frc.robot.subsystems.pneumatics.cannon;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class CannonIOHardware implements CannonIO {

  private final Solenoid solenoid;

  public CannonIOHardware(int solenoidChannel) {
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, solenoidChannel);
  }

  @Override
  public void updateInputs(CannonIOInputs inputs) {
    inputs.isOpen = solenoid.get();
  }

  @Override
  public void open() {
    // SmartDashboard.putBoolean("Cannon", false);
    solenoid.set(true);
  }

  @Override
  public void close() {
    // SmartDashboard.putBoolean("Cannon", false);
    solenoid.set(false);
  }
}
