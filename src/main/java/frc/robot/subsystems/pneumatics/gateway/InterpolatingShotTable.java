package frc.robot.subsystems.pneumatics.gateway;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolatingShotTable {

  private final InterpolatingDoubleTreeMap distanceToPressureMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap pressureToDistanceMap = new InterpolatingDoubleTreeMap();

  public InterpolatingShotTable() {
    // Values are intermediate tank psi and distance launched in meters
    // TODO enter real values
    addDataEntry(0, 0.0);
    addDataEntry(100, 50);
  }

  /**
   * Add a new entry to the table
   *
   * @param pressure tank pressure in psi when projectile was launched
   * @param distance distance in meters that projectile was launched
   */
  private void addDataEntry(double pressure, double distance) {
    pressureToDistanceMap.put(pressure, distance);
    distanceToPressureMap.put(distance, pressure);
  }

  /**
   * Get desired tank pressure in psi to launch projectile a certain distance
   *
   * @param distance distance in meters
   */
  public double getDesiredPSI(double distance) {
    return distanceToPressureMap.get(distance);
  }

  /**
   * Get estimated distance in meters that projectile will be launched at a certain tank pressure in
   * psi
   *
   * @param pressure tank pressure in psi
   */
  public double getEstimatedLaunchDistance(double pressure) {
    return pressureToDistanceMap.get(pressure);
  }
}
