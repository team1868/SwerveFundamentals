package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Control;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.DriveModes;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;

public class TeleopSwerveCommand extends CommandBase {
  private final Drivetrain _drivetrain;
  private final Controlboard _controlboard;
  private DriveModes _driveMode;
  private double _maxSpeedMPS;
  private Rotation2d _maxAngularSpeedPS;

  private static final DriveModes DEFAULT_DRIVE_MODE = DriveModes.FIELD_RELATIVE;

  public TeleopSwerveCommand(
      Drivetrain drivetrain,
      Controlboard controlboard,
      DriveModes initialMode,
      double maxSpeedMPS,
      Rotation2d maxAngularSpeedPS
  ) {
    _drivetrain = drivetrain;
    _controlboard = controlboard;
    _driveMode = initialMode;
    _maxSpeedMPS = maxSpeedMPS;
    _maxAngularSpeedPS = maxAngularSpeedPS;

    addRequirements(drivetrain);
  }

  public TeleopSwerveCommand(
      Drivetrain drivetrain, Controlboard controlboard, DriveModes initialMode
  ) {
    this(
        drivetrain,
        controlboard,
        initialMode,
        Constants.CRobot.drive.control.defaultLimits.maxTranslationalVelocityMPS,
        Constants.CRobot.drive.control.defaultLimits.maxAngularVelocityPS
    );
  }

  public void teleopInit() {
    _driveMode = DEFAULT_DRIVE_MODE;
  }

  public boolean condActivateSnapToAngle(int anglePOV) {
    if (anglePOV != -1) {
      _driveMode = DriveModes.SNAP_TO_ANGLE;
      // note that the pov axis does not align, we'll need some sort of flat
      // offset here Our functional 0 will be right facing to align with
      // odometry (3 on a standard clock face) POV 0 is "forward" (12 on a
      // standard clock face)
      // _drivetrain.setSnapAngle(((double) -anglePOV) + _drivetrain.POVZeroOffsetDeg()); // odom
      // zero offset

      var deg = ((double) -anglePOV) + _drivetrain.POVZeroOffsetDeg();
      if (Math.abs(anglePOV) == 180.0 || Math.abs(anglePOV) == 0.0 || Math.abs(anglePOV) == 90.0
          || Math.abs(anglePOV) == 270.0) {
        _drivetrain.setSnapAngleDeg(deg);
      }
      return true;
    }
    return false;
  }

  public void setDefaultDriveMode() {
    _driveMode = DEFAULT_DRIVE_MODE;
  }

  @Override
  public void execute() {
    // this code should probably move, but here before the state machine
    // management is also ok while dpad is pressed, set drive mode to
    // SNAP_TO_ANGLE and give the drivetrain a new target angle
    int anglePOV = 0; // needs to be initialized, but it doesn't matter what it's initalized to
                      // right? bc anglePOV isn't used unless isPIDTuningMode

    double xAxis = _controlboard.getDriveX();
    double yAxis = _controlboard.getDriveY();
    double rAxis = _controlboard.getRotX();

    if (RobotAltModes.isPIDTuningMode) {
      anglePOV = _controlboard._xboxDrive.getHID().getPOV();
    }
    switch (_driveMode) {
      case ROBOT_CENTRIC:
      case FIELD_RELATIVE:
        condActivateSnapToAngle(_controlboard._xboxDrive.getHID().getPOV());
        // condActivateSnapToAngleCritical(_controlboard._xboxDrive.getHID().getAButtonPressed());

        if (RobotAltModes.isPIDTuningMode && anglePOV != -1) {
          _drivetrain.testSteer((double) anglePOV);
        }
        break;
      case SNAP_TO_ANGLE:
        if (rAxis > Control.STICK_DEADBAND || rAxis < -Control.STICK_DEADBAND) {
          _driveMode = DEFAULT_DRIVE_MODE;
        } else {
          condActivateSnapToAngle(_controlboard._xboxDrive.getHID().getPOV());
        }
        break;
      default:
        System.err.println("Unkown drive mode");
        _driveMode = DEFAULT_DRIVE_MODE;
        break;
    }

    // We need logic here or in the drive commands that allows for desaturation to occur
    // contextually, capping maximal turning speed when stationary but not allowing the drive
    // command to override it due to maximum rotational speed being capped lower relative to the
    // theoretical value
    _drivetrain.drive(xAxis, yAxis, rAxis, _driveMode, _maxSpeedMPS, _maxAngularSpeedPS);
  }
}
