package frc.robot.subsystems.reservoir;

import org.littletonrobotics.junction.AutoLog;

public interface ReservoirIO {
  @AutoLog
  public static class ReservoirIOInputs {
    double tankPSI = 0;
    boolean compressorRunning = false;
  }

  public default void setTargetPressure(double pressure) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ReservoirIOInputs inputs) {}
}
