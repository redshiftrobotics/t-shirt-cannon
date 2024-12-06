package frc.robot.subsystems.gateway;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolatingShotTable {

  private final InterpolatingDoubleTreeMap distancePsiMap;

  public InterpolatingShotTable() {
    distancePsiMap = new InterpolatingDoubleTreeMap();
    distancePsiMap.put(10.0, 100.0);
  }

  /**
   * Get desired psi for a given distance
   *
   * @param distance distance in meters
   */
  public void get(double distance) {
    distancePsiMap.get(distance);
  }
}
