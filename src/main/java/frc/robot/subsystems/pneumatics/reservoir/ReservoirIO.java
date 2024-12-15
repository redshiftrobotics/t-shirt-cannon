package frc.robot.subsystems.pneumatics.reservoir;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ReservoirIO {
  @AutoLog
  public static class ReservoirIOInputs {
    double tankPSI = 0;
    boolean compressorRunning = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ReservoirIOInputs inputs) {}

  /** Enables the compressor to start filling tank */
  public default void startCompressor() {}

  /** Disables the compressor to stop filling tank */
  public default void stopCompressor() {}

  /** Sets a supplier that tells the sim whether it is draining. */
  public default void setSimDrain(BooleanSupplier pressureConsumer) {}
}
