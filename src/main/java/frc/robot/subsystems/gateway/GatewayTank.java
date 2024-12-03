package frc.robot.subsystems.gateway;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GatewayTank extends SubsystemBase {
  private final GatewayIO io;
  private final GatewayIOInputsAutoLogged inputs = new GatewayIOInputsAutoLogged();

  private final BangBangController controller;

  public GatewayTank(GatewayIO io) {
    this.io = io;

    controller = new BangBangController(GatewayConstants.PSI_TOLERANCE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GatewayTank", inputs);

    double output = controller.calculate(inputs.tankPSI);
    
    if (output > 0) {
      io.beganFilling();
    } else {
      io.stoppedFilling();
    }
  }

  public boolean isFilling() {
    return inputs.isFilling;
  }

  public double getPressure() {
    return inputs.tankPSI;
  }

  /**
   * Sets the setpoint pressure that the compressor will try and bring the reservoir tank to. Units
   * are PSI (or more accurately lbf/in^2).
   *
   * @param psi setpoint pressure in pound per square inch (PSI)
   * @throws IllegalArgumentException if desired pressure is set to negative psi
   */
  public void setDesiredPressure(double psi) {
    if (psi < 0) {
      throw new IllegalArgumentException("Unable to set desired pressure to negative PSI");
    }
    if (psi > GatewayConstants.FULL_TANK_PSI) {
      throw new IllegalArgumentException("Unable to set desired pressure to above max PSI");
    }

    controller.setSetpoint(psi);
  }

  public void setDistanceToTarget() {

  }
}
