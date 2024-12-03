package frc.robot.subsystems.cannon;

public class CannonIOSim implements CannonIO {

  boolean isOpen = false;

  @Override
  public void updateInputs(CannonIOInputs inputs) {
    inputs.isOpen = false;
  }

  @Override
  public void open() {
    isOpen = true;
  }

  @Override
  public void close() {
    isOpen = false;
  }
}
