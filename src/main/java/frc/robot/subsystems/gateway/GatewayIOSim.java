package frc.robot.subsystems.gateway;

import frc.robot.Constants;

public class GatewayIOSim implements GatewayIO {

  private static final double PSI_CHANGE_PER_SECOND = 1.2;

  private boolean isFilling;
  private double psi;

  public GatewayIOSim() {
    isFilling = false;
    psi = GatewayConstants.AVERAGE_ATMOSPHERIC_PSI;
  }

  @Override
  public void updateInputs(GatewayIOInputs input) {
    input.isFilling = isFilling;
    input.tankPSI = psi;

    psi += isFilling ? PSI_CHANGE_PER_SECOND * Constants.LOOP_PERIOD_SECONDS : 0;
  }

  @Override
  public void beganFilling() {
    isFilling = true;
  }

  @Override
  public void stoppedFilling() {
    isFilling = false;
  }
}
