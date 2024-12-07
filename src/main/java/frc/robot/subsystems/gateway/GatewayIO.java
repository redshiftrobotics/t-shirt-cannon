package frc.robot.subsystems.gateway;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface GatewayIO {
  @AutoLog
  public static class GatewayIOInputs {
    double tankPSI = 0;
    boolean isFilling = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GatewayIOInputs inputs) {}

  /** Open valve to begin filling tank */
  public default void beganFilling() {}

  /** Close valve to stop filling tank */
  public default void stopFilling() {}

  /** Sets a supplier that tells the sim whether it is draining. */
  public default void setSimDrain(BooleanSupplier pressureConsumer) {}
}
