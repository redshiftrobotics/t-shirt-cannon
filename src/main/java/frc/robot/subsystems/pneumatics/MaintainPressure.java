package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pneumatics.reservoir.ReservoirTank;
import frc.robot.utility.ThresholdController;

public class MaintainPressure extends Command {

  private final ReservoirTank tank;

  private final ThresholdController controller;

  public MaintainPressure(ReservoirTank tank, double lowerThreshold, double upperThreshold) {

    this.tank = tank;
    controller = new ThresholdController(lowerThreshold, upperThreshold);

    addRequirements(tank);
  }

  @Override
  public void initialize() {
    controller.reset();
  }

  @Override
  public void execute() {
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
