package frc.robot.subsystems.reservoir;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.hardwareWrappers.CustomPneumaticHub;

public class ReservoirIOSolenoid implements ReservoirIO {

  private CustomPneumaticHub hub;
  private Solenoid solenoid;

  private BangBangController controller;

  public ReservoirIOSolenoid() {
    hub = new CustomPneumaticHub();
    solenoid = hub.makeSolenoid(8);

    controller = new BangBangController(ReservoirConstants.PSI_TOLERANCE);
  }

  @Override
  public void updateInputs(ReservoirIOInputs inputs) {
    inputs.tankPSI = hub.getPressure(0);
    inputs.compressorRunning = controller.calculate(inputs.tankPSI) > 0;

    solenoid.set(inputs.compressorRunning);
  }

  @Override
  public void setTargetPressure(double pressure) {
    controller.setSetpoint(pressure);
  }
}
