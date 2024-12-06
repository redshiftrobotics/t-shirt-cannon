package frc.robot.subsystems.cannon;

import org.littletonrobotics.junction.AutoLog;

public interface CannonIO {
  @AutoLog
  public static class CannonIOInputs {
    boolean isOpen;
  }

  public default void updateInputs(CannonIOInputs inputs) {}

  public default void open() {}

  public default void close() {}
}
