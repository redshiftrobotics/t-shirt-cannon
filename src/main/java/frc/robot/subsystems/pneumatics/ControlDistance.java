package frc.robot.subsystems.pneumatics;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pneumatics.gateway.GatewayTank;
import frc.robot.utility.ThresholdController;

public class ControlDistance extends Command {

  private final GatewayTank tank;

  private final ThresholdController controller;

  private final Supplier<Double> distanceChange;

  private double targetDistance;

  public ControlDistance(GatewayTank tank, Supplier<Double> distanceChange) {

    this.tank = tank;

    controller = new ThresholdController(0.0, 0.0);

    this.distanceChange = distanceChange;

    addRequirements(tank);
  }

  @Override
  public void initialize() {
    controller.reset();
  }

  @Override
  public void execute() {
    targetDistance += MathUtil.applyDeadband(distanceChange.get(), 0.1);

    // controller.setThresholds(targetDistance, );

    if (controller.calculate(tank.getPressure()) > 0) {
      tank.startFilling();
    } else {
      tank.stopFilling();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    tank.stopFilling();
  }
}
