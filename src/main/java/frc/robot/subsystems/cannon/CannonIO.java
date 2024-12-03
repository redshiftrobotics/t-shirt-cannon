package frc.robot.subsystems.cannon;

import org.littletonrobotics.junction.AutoLog;

public interface CannonIO {
  @AutoLog
  public static class CannonIOInputs {
    boolean isOpen;
  }

  public void updateInputs(CannonIOInputs inputs);

  public void open();

  public void close();
}
