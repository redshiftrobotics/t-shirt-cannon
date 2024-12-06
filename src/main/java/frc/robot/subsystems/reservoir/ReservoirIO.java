package frc.robot.subsystems.reservoir;

import org.littletonrobotics.junction.AutoLog;

public interface ReservoirIO {
  @AutoLog
  public static class ReservoirIOInputs {
    double tankPSI = 0;
    boolean compressorRunning = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ReservoirIOInputs inputs) {}

  public default void startCompressor() {}

  public default void stopCompressor() {}
}
