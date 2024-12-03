package frc.robot.subsystems.gateway;

import org.littletonrobotics.junction.AutoLog;

public interface GatewayIO {
  @AutoLog
  public static class GatewayIOInputs {
    double tankPSI;
    boolean isFilling;
  }

  public default void updateInputs(GatewayIOInputs inputs) {}

  public default void beganFilling() {}

  public default void stoppedFilling() {}
}
