package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.dashboard.DriverDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.drive.controllers.SwerveJoystickUtil;
import frc.robot.subsystems.led.BlinkenLEDPattern;
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.pneumatics.cannon.*;
import frc.robot.subsystems.pneumatics.gateway.*;
import frc.robot.subsystems.pneumatics.reservoir.*;
import frc.robot.utility.OverrideSwitch;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final LEDSubsystem led;

  // Controller
  private final CommandXboxController xbox = new CommandXboxController(0);

  private final ReservoirTank reservoirTank;
  private final GatewayTank gatewayTank;
  private final FiringTube firingTube;

  // Dashboard
  private final DriverDashboard dashboard = DriverDashboard.getInstance();
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedDashboardChooser<BlinkenLEDPattern> ledPatternChooser;

  // Alerts
  private final Alert tuningModeActiveAlert =
      new Alert("Tuning mode active, do not use in real event.", AlertType.kWarning);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    led = new LEDSubsystem(LEDConstants.PWM_PORTS);

    switch (Constants.getRobot()) {
      case CANNON_BOT:
        drive =
            new Drive(
                new GyroIOPigeon2(DriveConstants.GYRO_CAN_ID),
                new ModuleIOSparkMax(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSparkMax(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
        reservoirTank = new ReservoirTank(new ReservoirIOHardware());
        gatewayTank = new GatewayTank(new GatewayIOHardware());
        firingTube =
            new FiringTube(
                new CannonIOHardware(CannonConstants.MIDDLE_FIRING_TUBE_SOLENOID_CHANNEL), "Main");
        break;

      case SIM_BOT:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FRONT_LEFT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.FRONT_RIGHT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.BACK_LEFT_MODULE_CONFIG),
                new ModuleIOSim(DriveConstants.BACK_RIGHT_MODULE_CONFIG));
        reservoirTank = new ReservoirTank(new ReservoirIOSim());
        gatewayTank = new GatewayTank(new GatewayIOSim());
        firingTube = new FiringTube(new CannonIOSim(), "Main");
        break;

      case TEST_BOT:
        // Test robot, disable IO implementations for most
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        reservoirTank = new ReservoirTank(new ReservoirIOHardware());
        gatewayTank = new GatewayTank(new GatewayIOHardware());
        firingTube =
            new FiringTube(
                new CannonIOHardware(CannonConstants.MIDDLE_FIRING_TUBE_SOLENOID_CHANNEL), "Main");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        reservoirTank = new ReservoirTank(new ReservoirIO() {});
        gatewayTank = new GatewayTank(new GatewayIO() {});
        firingTube = new FiringTube(new CannonIO() {}, "Main");
        break;
    }

    // Pneumatics sim
    reservoirTank.setSimDrain(gatewayTank::isFilling);
    gatewayTank.setSimDrain(firingTube::isOpen);

    // Set default target pressure
    gatewayTank.setTargetPressure(ControlConstants.shotTankDefaultPressure);
    reservoirTank.setPressureThresholds(
        ControlConstants.reservoirMinThresholdPressure,
        ControlConstants.reservoirMaxThresholdPressure);

    firingTube.setFireRequirements(() -> !gatewayTank.isFilling() || gatewayTank.isBackfilling());
    firingTube.setFireTubeOpenDurationSeconds(ControlConstants.FIRE_TUBE_OPEN_TIME_SECONDS);

    // Configure autos
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());
    configureAutos(autoChooser);
    dashboard.addChooser(autoChooser);

    // Configure LEDs
    ledPatternChooser =
        new LoggedDashboardChooser<>(
            "LED Pattern Chooser", new SendableChooser<BlinkenLEDPattern>());
    configureLEDs(ledPatternChooser);
    dashboard.addChooser(ledPatternChooser);

    // Configure alert triggers
    configureAlertTriggers();

    // Configure sysids
    if (Constants.TUNING_MODE) {
      configureSysIds(autoChooser);
    }

    // Alerts for constants to avoid using them in competition
    if (Constants.TUNING_MODE) {
      tuningModeActiveAlert.set(true);
    }

    // Hide controller missing warnings for sim
    if (Constants.getMode() != Mode.REAL) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    initDashboard();

    // Configure the button bindings
    configureSingleController();
  }

  private void configureAlertTriggers() {
    // VERY IMPORTANT: the firing tube checks if gateway is closed before firing (unless backfilling
    // exception), so we better pause if the firing tube is requesting to fire
    new Trigger(firingTube::isOpen)
        .or(firingTube::isWaitingToFire)
        .debounce(0.1, DebounceType.kBoth)
        .whileTrue(
            gatewayTank
                .startEnd(gatewayTank::pause, gatewayTank::unpause)
                .withName("Pause: Firing Tube Open"));
  }

  private void configureSingleController() {

    final Trigger useFieldRelative =
        new Trigger(new OverrideSwitch(xbox.y(), OverrideSwitch.Mode.TOGGLE, true));

    dashboard.fieldRelativeSupplier = useFieldRelative::getAsBoolean;
    dashboard.angleDrivenSupplier =
        () ->
            Stream.of(xbox.povUp(), xbox.povDown(), xbox.povLeft(), xbox.povRight())
                .anyMatch(Trigger::getAsBoolean);

    // Default command
    drive.setDefaultCommand(
        drive
            .runEnd(
                () -> {
                  Translation2d translation =
                      SwerveJoystickUtil.getTranslationMetersPerSecond(
                          -xbox.getLeftY(),
                          -xbox.getLeftX(),
                          drive.getMaxLinearSpeedMetersPerSec());

                  double rotation =
                      SwerveJoystickUtil.getOmegaRadiansPerSecond(
                          -xbox.getRightX(), drive.getMaxAngularSpeedRadPerSec());

                  drive.setRobotSpeeds(
                      new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                      useFieldRelative.getAsBoolean());
                },
                drive::stop)
            .withName("Drive"));

    // Heading controlled drive
    final HeadingController headingController = new HeadingController(drive);
    xbox.rightStick()
        .debounce(0.1)
        .whileTrue(
            drive
                .runEnd(
                    () -> {
                      Translation2d translation =
                          SwerveJoystickUtil.getTranslationMetersPerSecond(
                              -xbox.getLeftY(),
                              -xbox.getLeftX(),
                              drive.getMaxLinearSpeedMetersPerSec());

                      Optional<Rotation2d> heading =
                          SwerveJoystickUtil.getHeadingDirection(
                              -xbox.getRightY(), -xbox.getRightX());

                      heading.ifPresent(headingController::setGoal);

                      double rotation = headingController.calculate();

                      drive.setRobotSpeeds(
                          new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                          useFieldRelative.getAsBoolean());
                    },
                    drive::stop)
                .beforeStarting(headingController::reset)
                .withName("Heading Drive"));

    // --- Safety Controls ---

    // Drive reset gyro. Debounced to prevent accidental gyro resets that could mess up field
    // relative. Used when field relative angle is obviously wrong and needs to be reset.
    xbox.start()
        .debounce(0.3)
        .onTrue(
            drive
                .runOnce(
                    () ->
                        drive.resetPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)))
                .andThen(rumble(0.3).withTimeout(0.25))
                .ignoringDisable(true)
                .withName("Reset Gyro Heading"));

    // Cancel all driving and pause autofill when B is held
    xbox.b()
        .or(RobotModeTriggers.disabled())
        .onTrue(drive.runOnce(drive::stop).withName("Stop and Cancel"))
        .onTrue(rumble(0).withTimeout(Constants.LOOP_PERIOD_SECONDS))
        .whileTrue(
            reservoirTank
                .startEnd(reservoirTank::pause, reservoirTank::unpause)
                .withName("Pause: Manuel Cancel (B Held)"))
        .whileTrue(
            gatewayTank
                .startEnd(gatewayTank::pause, gatewayTank::unpause)
                .withName("Pause: Manuel Cancel (B Held)"));

    // Stop and orient swerve modules to 0 degrees when B is held
    xbox.b()
        .debounce(1)
        .onTrue(rumble(0.3).withTimeout(0.25))
        .whileTrue(drive.run(drive::stopUsingForwardArrangement).withName("Stop and Orient"));

    // --- Cannon Pneumatics Controls ---

    // Fire!
    xbox.rightTrigger()
        .and(
            xbox.back()
                .negate()
                .or(
                    gatewayTank
                        ::isPressureWithinTolerance)) // if checking wait until pressure is good to
        // trigger fire
        .onTrue(firingTube.runOnce(firingTube::fire).withName("Fire"));

    // "Load Shirt" (really just set loaded to true, for LEDs)
    // This is also on "check" button for convenience
    xbox.back().onTrue(Commands.runOnce(firingTube::loadShirt).ignoringDisable(true));

    // Backfill enabled (this means that the shot will release air from the gateway and the
    // reservoir). Consider this an "overfill"/ extra power mode
    xbox.leftTrigger()
        .and(firingTube::isOpen)
        .whileTrue(
            gatewayTank
                .startEnd(gatewayTank::backfill, gatewayTank::stopBackfill)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Opened (Backfill): Left Trigger"));

    // --- Gateway Setpoint Pressure Controls ---

    final Function<DoubleSupplier, Command> setPressureCommand =
        (DoubleSupplier pressure) ->
            Commands.runOnce(() -> gatewayTank.setTargetPressure(pressure.getAsDouble()))
                .ignoringDisable(true)
                .withName("Set Pressure");

    // Set gateway pressure to default setpoint
    xbox.a().onTrue(setPressureCommand.apply(() -> ControlConstants.shotTankDefaultPressure));

    // Set gateway pressure to secondary setpoint
    xbox.x().onTrue(setPressureCommand.apply(() -> ControlConstants.shotTankSecondaryPressure));

    // Increase gateway pressure setpoint
    xbox.leftBumper()
        .onTrue(
            setPressureCommand.apply(
                () -> gatewayTank.getTargetPressure() - ControlConstants.shotTankPressureChange));

    // Decrease gateway pressure setpoint
    xbox.rightBumper()
        .onTrue(
            setPressureCommand.apply(
                () -> gatewayTank.getTargetPressure() + ControlConstants.shotTankPressureChange));

    // --- Rumble Feedback ---

    // Give short rumble when pressure is good
    new Trigger(gatewayTank::isPressureWithinTolerance)
        .debounce(0.3, DebounceType.kFalling)
        .onTrue(rumble(0.3).withTimeout(0.5));

    // Check button (when held, the controller will rumble if good)
    xbox.back().whileTrue(rumble(0.05).onlyIf(gatewayTank::isPressureWithinTolerance));

    // Rumble when firing.
    new Trigger(firingTube::isOpen)
        .whileTrue(
            Commands.runEnd(
                    () -> {
                      double leftStrength =
                          MathUtil.inverseInterpolate(
                              GatewayConstants.MIN_ALLOWED_PRESSURE,
                              GatewayConstants.MAX_ALLOWED_PRESSURE,
                              gatewayTank.getPressure());
                      double rightStrength =
                          gatewayTank.isBackfilling()
                              ? MathUtil.inverseInterpolate(
                                  ReservoirConstants.MIN_ALLOWED_PRESSURE,
                                  ReservoirConstants.MAX_ALLOWED_PRESSURE,
                                  reservoirTank.getPressure())
                              : 0;

                      xbox.setRumble(RumbleType.kLeftRumble, leftStrength);
                      xbox.setRumble(RumbleType.kRightRumble, rightStrength);
                    },
                    () -> xbox.setRumble(RumbleType.kBothRumble, 0))
                .withName("Firing Rumble"));

    // --- POV Robot Strafing ---

    for (int i = 0; i < 360; i += 45) {
      final Rotation2d angle = Rotation2d.fromDegrees(-i);

      final ChassisSpeeds speeds =
          new ChassisSpeeds(
              Units.feetToMeters(2) * angle.getCos(),
              Units.feetToMeters(2) * angle.getSin(),
              Units.degreesToRadians(50) * angle.getSin());

      final String name = String.format("Robot Strafe %s\u00B0", angle.unaryMinus().getDegrees());

      xbox.pov(i)
          .debounce(0.2)
          .whileTrue(
              drive.run(() -> drive.setRobotSpeeds(speeds)).finallyDo(drive::stop).withName(name));
    }
  }

  private Command rumble(double value) {
    return Commands.startEnd(
            () -> xbox.setRumble(RumbleType.kBothRumble, value),
            () -> xbox.setRumble(RumbleType.kBothRumble, 0))
        .ignoringDisable(true);
  }

  /** Configure drive dashboard object */
  private void initDashboard() {
    dashboard.addSubsystem(drive);

    dashboard.poseSupplier = drive::getPose;
    dashboard.speedsSupplier = drive::getRobotSpeeds;

    dashboard.reservoirTankFilling = reservoirTank::isFilling;
    dashboard.reservoirTankPressure = reservoirTank::getPressure;
    dashboard.reservoirTankStatus = reservoirTank::getStatusString;

    dashboard.gatewayTankFilling = gatewayTank::isFilling;
    dashboard.gatewayTankPressure = gatewayTank::getPressure;
    dashboard.gatewayTankStatus = gatewayTank::getStatusString;

    dashboard.readyToFireSupplier = gatewayTank::isPressureWithinTolerance;
    dashboard.targetPressure = gatewayTank::getTargetPressure;
    dashboard.backfillMode = gatewayTank::isBackfilling;
    dashboard.cannonOpen = firingTube::isOpen;

    dashboard.addCommand("Reset Pose", drive.runOnce(() -> drive.resetPose(new Pose2d())), true);
    dashboard.addCommand(
        "Reset Rotation",
        drive.runOnce(
            () -> drive.resetPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))),
        true);

    dashboard.addCommand(
        "Stop Autofill (Dashboard)",
        Commands.parallel(
            reservoirTank.startEnd(reservoirTank::pause, reservoirTank::unpause),
            gatewayTank.startEnd(gatewayTank::pause, gatewayTank::unpause)),
        true);
  }

  private void configureLEDs(LoggedDashboardChooser<BlinkenLEDPattern> ledPatternChooser) {
    ledPatternChooser.addDefaultOption(
        String.format("Default (%s)", ControlConstants.idlePattern), ControlConstants.idlePattern);

    for (BlinkenLEDPattern pattern : BlinkenLEDPattern.values()) {
      ledPatternChooser.addOption(pattern.toString(), pattern);
    }

    led.setDefaultCommand(led.applyColor(ledPatternChooser::get).withName("LED Idle"));

    new Trigger(firingTube::isShirtLoaded)
        .whileTrue(led.applyColor(ControlConstants.loadedPattern).withName("LED Loaded"));
    new Trigger(firingTube::isOpen)
        .whileTrue(led.applyColor(ControlConstants.firingPattern).withName("LED Firing"));
  }

  private void configureAutos(LoggedDashboardChooser<Command> autoChooser) {
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "None + Stop Autofill",
        Commands.parallel(
                reservoirTank.startEnd(reservoirTank::pause, reservoirTank::unpause),
                gatewayTank.startEnd(gatewayTank::pause, gatewayTank::unpause))
            .withName("Stop Autofill (Auto)"));
    autoChooser.addOption(
        "Spin CCW",
        drive.runEnd(
            () -> drive.setRobotSpeeds(new ChassisSpeeds(0, 0, Units.degreesToRadians(45))),
            drive::stop));
    autoChooser.addOption(
        "Spin CW",
        drive.runEnd(
            () -> drive.setRobotSpeeds(new ChassisSpeeds(0, 0, Units.degreesToRadians(-45))),
            drive::stop));
    autoChooser.addOption(
        "Slow Spin CCW",
        drive.runEnd(
            () -> drive.setRobotSpeeds(new ChassisSpeeds(0, 0, Units.degreesToRadians(15))),
            drive::stop));
    autoChooser.addOption(
        "Slow Spin CW",
        drive.runEnd(
            () -> drive.setRobotSpeeds(new ChassisSpeeds(0, 0, Units.degreesToRadians(-15))),
            drive::stop));
  }

  private void configureSysIds(LoggedDashboardChooser<Command> autoChooser) {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
