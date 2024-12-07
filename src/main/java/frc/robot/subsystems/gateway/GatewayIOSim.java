package frc.robot.subsystems.gateway;

import frc.robot.Constants;

public class GatewayIOSim implements GatewayIO {

  private static final double COMPRESSOR_PSI_CHANGE_PER_SECOND = 5;
  private static final double PASSIVE_PSI_LEAK_PER_SECOND = 0.02;

  private double tankPSI;
  private boolean isFilling;

  public GatewayIOSim() {
    tankPSI = 0;
    isFilling = false;
  }

  @Override
  public void updateInputs(GatewayIOInputs inputs) {
    inputs.tankPSI = tankPSI;
    inputs.isFilling = isFilling;

    if (inputs.isFilling) {
      tankPSI += COMPRESSOR_PSI_CHANGE_PER_SECOND * Constants.LOOP_PERIOD_SECONDS;
    }
    tankPSI = Math.max(tankPSI - PASSIVE_PSI_LEAK_PER_SECOND * Constants.LOOP_PERIOD_SECONDS, 0);
  }

  @Override
  public void beganFilling() {
    isFilling = true;
  }

  @Override
  public void stopFilling() {
    isFilling = false;
  }
}
