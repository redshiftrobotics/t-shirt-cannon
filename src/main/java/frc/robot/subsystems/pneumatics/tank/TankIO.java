package frc.robot.subsystems.pneumatics.tank;

import org.littletonrobotics.junction.AutoLog;

public interface TankIO {
  @AutoLog
  public static class TankIOInputs {
    double tankPSI = 0;
    boolean isFilling = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TankIOInputs inputs) {}

  /** Enables the system to start filling tank */
  public default void enable() {}

  /** Disables the system to stop filling tank */
  public default void disable() {}
}
