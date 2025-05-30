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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
import frc.robot.subsystems.drive.controllers.SpeedController;
import frc.robot.subsystems.drive.controllers.SpeedController.SpeedLevel;
import frc.robot.subsystems.drive.controllers.TeleopDriveController;
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.subsystems.led.LEDConstants.LEDPatterns;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.pneumatics.cannon.*;
import frc.robot.subsystems.pneumatics.gateway.*;
import frc.robot.subsystems.pneumatics.reservoir.*;
import frc.robot.utility.JoystickUtil;
import frc.robot.utility.OverrideSwitch;
import java.util.Optional;
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
  private static final boolean ONE_CONTROLLER = true;
  private final CommandGenericHID driverController = new CommandXboxController(0);
  private final CommandGenericHID operatorController =
      ONE_CONTROLLER ? null : new CommandXboxController(1);

  private final ReservoirTank reservoirTank;
  private final GatewayTank gatewayTank;
  private final FiringTube firingTube;

  // Dashboard
  private final DriverDashboard dashboard = DriverDashboard.getInstance();
  private final LoggedDashboardChooser<Command> autoChooser;

  // Alerts
  private final Alert tuningModeActiveAlert =
      new Alert("Tuning mode active, do not use in real event.", AlertType.kWarning);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    led = new LEDSubsystem(LEDConstants.PWM_PORT, LEDPatterns.GREEN);

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

    gatewayTank.setTargetPressure(ControlConstants.shotTankDefaultPressure);

    reservoirTank.setPressureThresholds(
        ControlConstants.reservoirMinThresholdPressure,
        ControlConstants.reservoirMaxThresholdPressure);
    firingTube.setFireRequirements(() -> !gatewayTank.isFilling() || gatewayTank.isBackfilling());

    // Can also use AutoBuilder.buildAutoChooser(); instead of SendableChooser to
    // auto populate
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", new SendableChooser<Command>());

    // Configure autos
    configureAutos();

    // Configure sysids
    if (Constants.TUNING_MODE) {
      configureSysIds();
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
    configureControllerBindings();
  }

  /** Define button->command mappings. */
  private void configureControllerBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    if (ONE_CONTROLLER) {
      configureSingleController();
    } else {
      configureDriverControllerBindings();
      configureOperatorControllerBindings();
    }
    configureAlertTriggers();
  }

  private void configureAlertTriggers() {
    // RobotModeTriggers.teleop().onChange(rumbleControllers(0.2).withTimeout(0.1));

    // new Trigger(
    //         () ->
    //             NormUtil.norm(drive.getRobotSpeeds()) > drive.getMaxLinearSpeedMetersPerSec() *
    // 0.9
    //                 || Math.abs(drive.getRobotSpeeds().omegaRadiansPerSecond)
    //                     > drive.getMaxAngularSpeedRadPerSec() * 0.9)
    //     .debounce(0.3, DebounceType.kBoth)
    //     .whileTrue(
    //         reservoirTank
    //             .startEnd(reservoirTank::pause, reservoirTank::unpause)
    //             .withName("Pause: High Drive Speed"));

    // VERY IMPORTANT: the firing tube checks if gateway is closed before firing (unless backfilling
    // exception), so we better pause if the firing tube is requesting to fire
    new Trigger(firingTube::isOpen)
        .or(firingTube::isWaitingToFire)
        .debounce(0.1, DebounceType.kBoth)
        .whileTrue(
            gatewayTank
                .startEnd(gatewayTank::pause, gatewayTank::unpause)
                .withName("Pause: Firing Tube Open"));

    new Trigger(firingTube::isShirtLoaded).whileTrue(led.solid(LEDPatterns.RED));

    new Trigger(firingTube::isOpen)
        .whileTrue(led.flash(LEDPatterns.YELLOW, LEDPatterns.WHITE, 0.2));
  }

  private void configureSingleController() {

    final CommandXboxController xbox = (CommandXboxController) driverController;

    final Trigger useFieldRelative =
        new Trigger(new OverrideSwitch(xbox.y(), OverrideSwitch.Mode.TOGGLE, true));

    // Controllers
    final TeleopDriveController input =
        new TeleopDriveController(
            drive,
            () -> -xbox.getLeftY(),
            () -> -xbox.getLeftX(),
            () -> -xbox.getRightY(),
            () -> -xbox.getRightX());

    DriverDashboard.getInstance().setSpeedLevelSupplier(() -> SpeedController.SpeedLevel.NO_LEVEL);
    DriverDashboard.getInstance()
        .setAngleDrivenSupplier(
            () ->
                Stream.of(xbox.povUp(), xbox.povDown(), xbox.povLeft(), xbox.povRight())
                    .anyMatch(Trigger::getAsBoolean));
    DriverDashboard.getInstance().setFieldRelativeSupplier(useFieldRelative);

    // Default command
    drive.setDefaultCommand(
        drive
            .runEnd(
                () -> {
                  Translation2d translation = input.getTranslationMetersPerSecond();
                  double rotation = input.getOmegaRadiansPerSecond();
                  drive.setRobotSpeeds(
                      new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
                      useFieldRelative.getAsBoolean());
                },
                drive::stop)
            .withName("DefaultDrive"));

    // Drive reset gyro
    xbox.start()
        .debounce(0.3)
        .onTrue(
            drive
                .runOnce(
                    () ->
                        drive.resetPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)))
                .andThen(
                    Commands.runEnd(
                            () -> rumbleController(xbox, 0.3), () -> rumbleController(xbox, 0))
                        .withTimeout(0.25))
                .ignoringDisable(true)
                .withName("Reset Gyro Heading"));

    // Drive cancel all commands
    xbox.b()
        .or(RobotModeTriggers.disabled())
        .onTrue(drive.runOnce(drive::stop).withName("StopCancel"))
        .onTrue(Commands.runOnce(() -> rumbleController(xbox, 0)))
        .whileTrue(
            reservoirTank
                .startEnd(reservoirTank::pause, reservoirTank::unpause)
                .withName("Pause: Manuel Cancel (B Held)"))
        .whileTrue(
            gatewayTank
                .startEnd(gatewayTank::pause, gatewayTank::unpause)
                .withName("Pause: Manuel Cancel (B Held)"));

    // Fire!
    xbox.rightTrigger()
        .and(() -> !xbox.back().getAsBoolean() || gatewayTank.isPressureWithinTolerance())
        .onTrue(firingTube.runOnce(firingTube::fire).withName("Fire"));

    xbox.back().onTrue(Commands.runOnce(firingTube::loadShirt));

    // Backfill enabled
    xbox.leftTrigger()
        .and(firingTube::isOpen)
        .whileTrue(
            gatewayTank
                .startEnd(gatewayTank::backfill, gatewayTank::stopBackfill)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Opened (Backfill): Left Trigger"));

    // Vibrations
    // Really need some LED for this
    new Trigger(gatewayTank::isPressureWithinTolerance)
        .debounce(0.3, DebounceType.kFalling)
        .onTrue(
            Commands.startEnd(() -> rumbleController(xbox, 0, 0.3), () -> rumbleController(xbox, 0))
                .withTimeout(0.5));

    xbox.back()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  rumbleController(xbox, gatewayTank.isPressureWithinTolerance() ? 0.05 : 0);
                },
                () -> rumbleController(xbox, 0)));

    // Rumble when firing
    new Trigger(firingTube::isOpen)
        .whileTrue(
            Commands.runEnd(
                    () -> {
                      rumbleController(
                          xbox,
                          MathUtil.inverseInterpolate(
                              GatewayConstants.MIN_ALLOWED_PRESSURE,
                              GatewayConstants.MAX_ALLOWED_PRESSURE,
                              gatewayTank.getPressure()),
                          gatewayTank.isBackfilling()
                              ? MathUtil.inverseInterpolate(
                                  ReservoirConstants.MIN_ALLOWED_PRESSURE,
                                  ReservoirConstants.MAX_ALLOWED_PRESSURE,
                                  reservoirTank.getPressure())
                              : 0);
                    },
                    () -> rumbleController(xbox, 0))
                .withName("Firing Rumble"));

    // Gateway default pressure
    xbox.a()
        .onTrue(
            Commands.runOnce(
                () -> gatewayTank.setTargetPressure(ControlConstants.shotTankDefaultPressure)));

    // Gateway secondary pressure
    xbox.x()
        .onTrue(
            Commands.runOnce(
                () -> gatewayTank.setTargetPressure(ControlConstants.shotTankSecondaryPressure)));

    // Gateway pressure change
    xbox.leftBumper()
        .onTrue(
            Commands.runOnce(
                () ->
                    gatewayTank.setTargetPressure(
                        gatewayTank.getTargetPressure()
                            - ControlConstants.shotTankPressureChange)));

    xbox.rightBumper()
        .onTrue(
            Commands.runOnce(
                () ->
                    gatewayTank.setTargetPressure(
                        gatewayTank.getTargetPressure()
                            + ControlConstants.shotTankPressureChange)));

    // Dpad robot relative drive
    xbox.povLeft()
        .whileTrue(
            drive.startEnd(() -> drive.setRobotSpeeds(new ChassisSpeeds(0, 2, 3)), drive::stop));

    xbox.povRight()
        .whileTrue(
            drive.startEnd(() -> drive.setRobotSpeeds(new ChassisSpeeds(0, -2, -3)), drive::stop));

    xbox.povUp()
        .whileTrue(
            drive.startEnd(() -> drive.setRobotSpeeds(new ChassisSpeeds(2, 0, 0)), drive::stop));

    xbox.povDown()
        .whileTrue(
            drive.startEnd(() -> drive.setRobotSpeeds(new ChassisSpeeds(-2, 0, 0)), drive::stop));
  }

  private static void rumbleController(CommandGenericHID controller, double rumbleIntensity) {
    controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    SmartDashboard.putNumber("LeftControllerRumble", rumbleIntensity);
    SmartDashboard.putNumber("RightControllerRumble", rumbleIntensity);
  }

  private static void rumbleController(
      CommandGenericHID controller, double leftIntensity, double rightIntensity) {
    controller.setRumble(RumbleType.kLeftRumble, leftIntensity);
    SmartDashboard.putNumber("LeftControllerRumble", leftIntensity);
    controller.setRumble(RumbleType.kRightRumble, rightIntensity);
    SmartDashboard.putNumber("RightControllerRumble", rightIntensity);
  }

  /** Configure drive dashboard object */
  private void initDashboard() {
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());

    dashboard.addSubsystem(drive);
    dashboard.setPoseSupplier(drive::getPose);
    dashboard.setRobotSpeedsSupplier(drive::getRobotSpeeds);
    dashboard.setSpeedLevelSupplier(() -> SpeedController.SpeedLevel.NO_LEVEL);
    dashboard.setFieldRelativeSupplier(() -> false);
    dashboard.setAngleDrivenSupplier(() -> false);

    dashboard.setReservoirTank(
        reservoirTank::isFilling, reservoirTank::getPressure, reservoirTank::getStatusString);

    dashboard.setGatewayTank(
        gatewayTank::isFilling, gatewayTank::getPressure, gatewayTank::getStatusString);

    dashboard.setCannon(
        gatewayTank::isPressureWithinTolerance,
        gatewayTank::getTargetPressure,
        gatewayTank::isBackfilling,
        firingTube::isOpen);

    dashboard.addCommand("Reset Pose", drive.runOnce(() -> drive.resetPose(new Pose2d())), true);
    dashboard.addCommand(
        "Reset Rotation",
        drive.runOnce(
            () -> drive.resetPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))),
        true);
  }

  private void configureDriverControllerBindings() {
    if (driverController instanceof CommandXboxController) {
      final CommandXboxController driverXbox = (CommandXboxController) driverController;

      final Trigger useFieldRelative =
          new Trigger(new OverrideSwitch(driverXbox.y(), OverrideSwitch.Mode.TOGGLE, true));

      final Trigger useAngleControlMode =
          new Trigger(
              new OverrideSwitch(driverXbox.rightBumper(), OverrideSwitch.Mode.HOLD, false));

      // Controllers
      final TeleopDriveController input =
          new TeleopDriveController(
              drive,
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX(),
              () -> -driverXbox.getRightY(),
              () -> -driverXbox.getRightX());

      final SpeedController speedController = new SpeedController(SpeedLevel.DEFAULT);

      final HeadingController headingController = new HeadingController(drive);

      DriverDashboard.getInstance().setSpeedLevelSupplier(speedController::getCurrentSpeedLevel);
      DriverDashboard.getInstance().setAngleDrivenSupplier(useAngleControlMode);
      DriverDashboard.getInstance().setFieldRelativeSupplier(useFieldRelative);

      // Default command
      drive.setDefaultCommand(
          drive
              .runEnd(
                  () -> {
                    Translation2d translation = input.getTranslationMetersPerSecond();
                    double rotation = input.getOmegaRadiansPerSecond();
                    drive.setRobotSpeeds(
                        speedController.updateSpeed(
                            new ChassisSpeeds(translation.getX(), translation.getY(), rotation)),
                        useFieldRelative.getAsBoolean());
                  },
                  drive::stop)
              .withName("DefaultDrive"));

      useAngleControlMode
          .debounce(0.1)
          .onTrue(
              Commands.runOnce(
                      () -> {
                        headingController.reset();
                        headingController.setGoal(drive.getPose().getRotation());
                      })
                  .withName("PrepareAngleDrive"))
          .whileTrue(
              drive
                  .runEnd(
                      () -> {
                        Translation2d translation = input.getTranslationMetersPerSecond();
                        Optional<Rotation2d> rotation = input.getHeadingDirection();
                        rotation.ifPresent(headingController::setGoal);
                        double omegaRadiansPerSecond = headingController.calculate();
                        drive.setRobotSpeeds(
                            speedController.updateSpeed(
                                new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    headingController.atGoal() ? 0 : omegaRadiansPerSecond)),
                            useFieldRelative.getAsBoolean());
                      },
                      drive::stop)
                  .withName("RotationAngleDrive"));

      boolean includeDiagonalPOV = true;
      for (int pov = 0; pov < 360; pov += includeDiagonalPOV ? 45 : 90) {

        // POV angles are in Clock Wise degrees, needs to be flipped to get correct
        // rotation2d
        final Rotation2d angle = Rotation2d.fromDegrees(-pov);
        final String name = String.format("%d\u00B0", pov);

        // While the POV is being pressed and we are not in angle control mode, set the
        // chassis speeds to the Cos and Sin of the angle
        driverXbox
            .pov(pov)
            .and(useAngleControlMode)
            .whileTrue(
                drive
                    .runEnd(
                        () ->
                            drive.setRobotSpeeds(
                                speedController.updateSpeed(
                                    new ChassisSpeeds(angle.getCos(), angle.getSin(), 0))),
                        drive::stop)
                    .withName(String.format("DriveRobotRelative %s", name)));

        // While the POV is being pressed and we are angle control mode
        // Start by resetting the controller and setting the goal angle to the pov angle
        driverXbox
            .pov(pov)
            .and(useAngleControlMode.negate())
            .onTrue(
                drive
                    .runOnce(
                        () -> {
                          headingController.reset();
                          headingController.setGoal(angle);
                        })
                    .withName(String.format("PrepareLockedHeading %s", name)));

        // Then if the button is held for more than 0.2 seconds, drive forward at the
        // angle once the chassis reaches it
        driverXbox
            .pov(pov)
            .debounce(0.2)
            .and(useAngleControlMode.negate())
            .whileTrue(
                drive
                    .run(
                        () -> {
                          double rotationRadians = headingController.calculate();
                          drive.setRobotSpeeds(
                              speedController.updateSpeed(
                                  new ChassisSpeeds(
                                      (headingController.atGoal() ? 1 : 0),
                                      0,
                                      headingController.atGoal() ? 0 : rotationRadians)));
                        })
                    .withName(String.format("ForwardLockedHeading %s", name)));

        // Then once the pov is let go, if we are not at the angle continue turn to it,
        // while also accepting x and y input to drive. Cancel once we get turn request
        driverXbox
            .pov(pov)
            .and(useAngleControlMode.negate())
            .onFalse(
                drive
                    .run(
                        () -> {
                          Translation2d translation = input.getTranslationMetersPerSecond();
                          double rotationRadians = headingController.calculate();
                          drive.setRobotSpeeds(
                              speedController.updateSpeed(
                                  new ChassisSpeeds(
                                      translation.getX(),
                                      translation.getY(),
                                      headingController.atGoal() ? 0 : rotationRadians)),
                              useFieldRelative.getAsBoolean());
                        })
                    .until(() -> input.getOmegaRadiansPerSecond() != 0)
                    .withName(String.format("DriveLockedHeading %s", name)));
      }

      // While X is held down go into stop and go into the cross position to resistant
      // movement,
      // then once X button is let go put modules forward
      driverXbox
          .x()
          .whileTrue(
              drive
                  .startEnd(drive::stopUsingBrakeArrangement, drive::stopUsingForwardArrangement)
                  .withName("StopWithX"));

      // Reset Rotation
      driverXbox
          .start()
          .onTrue(
              Commands.runOnce(
                      () ->
                          drive.resetPose(
                              new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)))
                  .withName("Reset Rotation"));

      // When be is pressed stop the drivetrain then idle it, cancelling all incoming
      // commands.
      // Also do this when robot is disabled
      driverXbox
          .b()
          .or(RobotModeTriggers.disabled())
          .onTrue(drive.runOnce(drive::stop).withName("StopCancel"));

      // When right (accelerator/fire) trigger is pressed, put in boost (fast) mode
      driverXbox
          .rightTrigger(0.5)
          .whileTrue(
              Commands.startEnd(
                  () -> speedController.pushSpeedLevel(SpeedLevel.BOOST),
                  () -> speedController.removeSpeedLevel(SpeedLevel.BOOST)));

      // When left (brake/ADS) trigger is is pressed put in precise (slow) mode
      driverXbox
          .leftTrigger(0.5)
          .whileTrue(
              Commands.startEnd(
                  () -> speedController.pushSpeedLevel(SpeedLevel.PRECISE),
                  () -> speedController.removeSpeedLevel(SpeedLevel.PRECISE)));

    } else if (driverController instanceof CommandJoystick) {
      final CommandJoystick driverJoystick = (CommandJoystick) driverController;

      TeleopDriveController input =
          new TeleopDriveController(
              drive,
              () -> -driverJoystick.getY(),
              () -> -driverJoystick.getX(),
              () -> -driverJoystick.getTwist(),
              () -> 0);

      drive.setDefaultCommand(
          drive.startEnd(
              () -> {
                Translation2d translation = input.getTranslationMetersPerSecond();
                double rotation = input.getOmegaRadiansPerSecond();
                drive.setRobotSpeeds(
                    new ChassisSpeeds(translation.getX(), translation.getY(), rotation), false);
              },
              drive::stop));
    }
  }

  private void configureOperatorControllerBindings() {
    if (operatorController instanceof CommandXboxController) {
      final CommandXboxController operatorXbox = (CommandXboxController) operatorController;

      // TODO set up rest of control code in here. use default command to fill it 20,
      // then have
      // another command that idles it till it reaches 15
      // Just display default command (fill to 20) and current command (pauses, etc)
      // using subsystem

      // Set up reservoir tank

      operatorXbox
          .y()
          .whileTrue(
              reservoirTank
                  .startEnd(reservoirTank::pause, reservoirTank::unpause)
                  .withName("Pause: Operator Y Button"));

      operatorXbox
          .leftTrigger()
          .whileTrue(
              gatewayTank
                  .startEnd(gatewayTank::pause, gatewayTank::unpause)
                  .withName("Pause: Operator Prepare Fire"));

      new Trigger(firingTube::isOpen).whileTrue(rumbleControllers(0.5));

      operatorXbox
          .a()
          .or(operatorXbox.x())
          .and(firingTube::isOpen)
          .whileTrue(
              gatewayTank
                  .startEnd(gatewayTank::backfill, gatewayTank::stopBackfill)
                  .withName("Opened (Backfill): Operator A/X Button"));

      operatorXbox.rightTrigger().onTrue(firingTube.runOnce(firingTube::fire).withName("Fire"));

      operatorXbox
          .povUp()
          .onTrue(
              gatewayTank
                  .runOnce(
                      () ->
                          gatewayTank.setTargetPressure(
                              Math.floor(gatewayTank.getTargetPressure() + 1)))
                  .withName("Increase Target PSI"));

      operatorXbox
          .povDown()
          .onTrue(
              gatewayTank
                  .runOnce(
                      () ->
                          gatewayTank.setTargetPressure(
                              Math.ceil(gatewayTank.getTargetPressure() - 1)))
                  .withName("Decrease Target PSI"));

      gatewayTank.setDefaultCommand(
          gatewayTank
              .run(
                  () -> {
                    gatewayTank.setTargetPressure(
                        gatewayTank.getTargetPressure()
                            - JoystickUtil.applyDeadband(operatorXbox.getLeftY())
                                * Constants.LOOP_PERIOD_SECONDS
                                * 5);
                  })
              .withName("Adjust Target PSI"));
      // gatewayTank.setDefaultCommand(
      // gatewayTank
      // .run(
      // () -> {
      // gatewayTank.setTargetLaunchDistance(
      // gatewayTank.getTargetLaunchDistance()
      // - MathUtil.applyDeadband(operatorXbox.getLeftY(), 0.1) * 0.4);
      // })
      // .withName("Adjust Target Distance"));
    }
  }

  private Command rumbleControllers(double rumbleIntensity) {
    return Commands.startEnd(
            () -> {
              driverController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
              if (operatorController != null) {
                operatorController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
              }
            },
            () -> {
              driverController.setRumble(RumbleType.kBothRumble, 0);
              if (operatorController != null) {
                operatorController.setRumble(RumbleType.kBothRumble, 0);
              }
            })
        .withName("RumbleController");
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("None", Commands.none());
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

  private void configureSysIds() {
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
