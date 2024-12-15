package frc.robot.subsystems.pneumatics.cannon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.stream.Stream;

/**
 * A subsystem that controls many firing tubes. This is not complete, and for when we have 3 tubes
 * on the robot
 */
public class FiringSystem extends SubsystemBase {

  public final FiringTube[] tubes;

  /** Creates a new FiringSystem subsystem */
  public FiringSystem(FiringTube... tubes) {
    this.tubes = tubes;
  }

  @Override
  public void periodic() {
    tubes().forEach(FiringTube::periodic);
  }

  public boolean isOpen() {
    return tubes().allMatch(FiringTube::isOpen);
  }

  private Stream<FiringTube> tubes() {
    return Arrays.stream(tubes);
  }
}
