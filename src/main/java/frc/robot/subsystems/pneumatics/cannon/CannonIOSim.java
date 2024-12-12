package frc.robot.subsystems.pneumatics.cannon;

public class CannonIOSim implements CannonIO {

  private boolean isOpen = false;

  @Override
  public void updateInputs(CannonIOInputs inputs) {
    inputs.isOpen = isOpen;
  }

  @Override
  public void open() {
    System.out.println("Whoosh!");
    isOpen = true;
  }

  @Override
  public void close() {
    isOpen = false;
  }
}
