package frc.robot.subsystems.reservoir;

import edu.wpi.first.math.controller.BangBangController;
import frc.robot.Constants;

public class ReservoirIOSim implements ReservoirIO {

  private static final double PSI_CHANGE_PER_SECOND = 1.2;

  private double tankPSI;

  private BangBangController controller;

  public ReservoirIOSim() {
    tankPSI = ReservoirConstants.AVERAGE_ATMOSPHERIC_PSI;
    controller = new BangBangController(ReservoirConstants.PSI_TOLERANCE);
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = tankPSI;
    inputs.compressorRunning = controller.calculate(inputs.tankPSI) > 0;

    tankPSI += inputs.compressorRunning ? PSI_CHANGE_PER_SECOND * Constants.LOOP_PERIOD_SECONDS : 0;
  }

  @Override
  public void setTargetPressure(double pressure) {
    controller.setSetpoint(pressure);
  }
}
