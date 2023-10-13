package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Sensors;
import frc.robot.constants.DrivetrainConfs;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.*;
import frc.robot.constants.enums.DriveModes;
import frc.robot.utils.InputUtils;

public class Drivetrain extends SubsystemBase {
  /* --- Constant configuration shortcuts --- */
  private final DrivetrainConfs DRIVE_CONSTS = Constants.CRobot.drive;
  private boolean disabled = DRIVE_CONSTS == null;
  private final DrivetrainDimensions DRIVE_DIMS = disabled ? null : DRIVE_CONSTS.dims;
  private final DrivetrainControl DRIVE_CONTROL = disabled ? null : DRIVE_CONSTS.control;
  private final DrivetrainPorts DRIVE_PORTS = disabled ? null : DRIVE_CONSTS.ports;

  private boolean wantOrientationMode = false;
  private final boolean USE_POSE_ESTIMATION_ANGLE = false; // change if true?

  /* --- Sensors, motors, and hardware --- */
  private WPI_Pigeon2 _gyro = new WPI_Pigeon2(DRIVE_PORTS.pigeonID, DRIVE_PORTS.pigeonCanBus);
  private Pigeon2Configuration _gyroConfigs;

  private SwerveModule[] _modules = new SwerveModule[] {
      new SwerveModule(0, DRIVE_PORTS.moduleCanBus),
      new SwerveModule(1, DRIVE_PORTS.moduleCanBus),
      new SwerveModule(2, DRIVE_PORTS.moduleCanBus),
      new SwerveModule(3, DRIVE_PORTS.moduleCanBus)};

  /* --- State and Physical Property variables --- */
  private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds();
  // Initialize with real values regardless of canbus state to ensure good launch to pose
  // estimator
  private SwerveModulePosition[] _modulePositions = new SwerveModulePosition[] {
      _modules[0].getPosition(),
      _modules[1].getPosition(),
      _modules[2].getPosition(),
      _modules[3].getPosition()};

  /* --- Odometry Utils --- */
  // TODO: move to configurations
  private edu.wpi.first.math.kinematics.SwerveDriveKinematics _swerveKinematics =
      new edu.wpi.first.math.kinematics.SwerveDriveKinematics(
          new Translation2d(DRIVE_DIMS.halfTrackLength_M, DRIVE_DIMS.halfTrackWidth_M),
          new Translation2d(DRIVE_DIMS.halfTrackLength_M, -DRIVE_DIMS.halfTrackWidth_M),
          new Translation2d(-DRIVE_DIMS.halfTrackLength_M, DRIVE_DIMS.halfTrackWidth_M),
          new Translation2d(-DRIVE_DIMS.halfTrackLength_M, -DRIVE_DIMS.halfTrackWidth_M)
      );

  SwerveDrivePoseEstimator _robotPoseEstimator = new SwerveDrivePoseEstimator(
      _swerveKinematics, Rotation2d.fromDegrees(0.0), _modulePositions, new Pose2d()
  );

  /* --- Control Utils --- */
  private ProfiledPIDController _xController = DRIVE_CONTROL.getTranslationaProfiledPIDController();
  private ProfiledPIDController _yController = DRIVE_CONTROL.getTranslationaProfiledPIDController();
  private ProfiledPIDController _angleController = DRIVE_CONTROL.getAngularProfiledPIDController();

  private SlewRateLimiter _xSlewRateFilter =
      DRIVE_CONTROL.slewingLimits.getTranslationalSlewRateLimiter();
  private SlewRateLimiter _ySlewRateFilter =
      DRIVE_CONTROL.slewingLimits.getTranslationalSlewRateLimiter();
  private SlewRateLimiter _angleSlewRateFilter =
      DRIVE_CONTROL.slewingLimits.getAngularSlewRateLimiter();

  /* --- Game State variables --- */
  private Field2d _field;
  private Alliance _alliance;
  private boolean _isAuto = false;
  private boolean _autoPrepScore = false;

  /* --- Simulation resources and variables --- */
  private Pose2d _simPose = new Pose2d();

  /* --- Logging variables --- */
  private double[] _desiredSwerveStates = new double[DRIVE_CONSTS.numModules * 2];
  private double[] _entry = new double[2 * DRIVE_CONSTS.numModules];

  /* --- Shuffleboard entries --- */
  private GenericEntry _desiredSpeed, _actualSpeed;
  private GenericEntry _actualSpeedX, _actualSpeedY, _actualSpeedTheta;
  private GenericEntry _desiredRobotTheta, _actualRobotTheta, _errorRobotTheta;

  private GenericPublisher _xPoseError, _yPoseError, _thetaPoseError;
  private GenericEntry _usingPoseTuning;

  private GenericEntry _anglePFac, _angleIFac, _angleDFac;
  private GenericEntry _xPFac, _xIFac, _xDFac;
  private GenericEntry _yPFac, _yIFac, _yDFac;

  private GenericEntry _steerTargetAngle, _steerErrorAngle;
  private GenericEntry _steerAnglePFac, _steerAngleIFac, _steerAngleDFac;
  private GenericEntry _steerPFac, _steerIFac, _steerDFac;

  private Pose2d TARGET_RELATIVE_POSE = new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0));

  public Drivetrain(Field2d field) {
    _field = field;

    resetDefaultTolerance();
    _angleController.enableContinuousInput(0.0, Units.degreesToRadians(360.0));

    if (Robot.isReal()) {
      int counter = 0;
      while (!checkInitStatus()) {
        System.out.println("DRIVETRAIN Check Init Status : " + counter++);
      }
    } else {
    }

    _gyro.configFactoryDefault();
    zeroGyro();
    configShuffleboard();
  }

  @Override
  public void periodic() {
    // get module positions ensures the order of data beign collected
    _robotPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

    updateShuffleboard();

    // TODO: call an update function and pre-construct the array
    _chassisSpeeds = _swerveKinematics.toChassisSpeeds(
        _modules[0].getState(),
        _modules[1].getState(),
        _modules[2].getState(),
        _modules[3].getState()
    );
  }

  public void updateModulePositions() {
    for (int i = 0; i < DRIVE_CONSTS.numModules; i++) {
      _modulePositions[i] = _modules[i].getPosition();
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    updateModulePositions();
    return _modulePositions;
  }

  public void enableAuto() {
    _isAuto = true;
  }

  public void disableAuto() {
    _isAuto = false;
  }

  public void onEnable() {
    if (RobotAltModes.isPIDTuningMode) {
      _angleController.setPID(
          _anglePFac.getDouble(DRIVE_CONTROL.theta.p),
          _angleIFac.getDouble(DRIVE_CONTROL.theta.i),
          _angleDFac.getDouble(DRIVE_CONTROL.theta.d)
      );
      _xController.setPID(
          _xPFac.getDouble(DRIVE_CONTROL.xy.p),
          _xIFac.getDouble(DRIVE_CONTROL.xy.i),
          _xDFac.getDouble(DRIVE_CONTROL.xy.d)
      );
      _yController.setPID(
          _yPFac.getDouble(DRIVE_CONTROL.xy.p),
          _yIFac.getDouble(DRIVE_CONTROL.xy.i),
          _yDFac.getDouble(DRIVE_CONTROL.xy.d)
      );
    }
  }

  public void configShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SwerveModules");
    _steerTargetAngle = tab.add("target angle", 0.0).getEntry();

    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

    if (RobotAltModes.isPoseTuning) {
      ShuffleboardLayout poseLayout = drivetrainTab.getLayout("Go To Pose", BuiltInLayouts.kGrid)
                                          .withSize(1, 2)
                                          .withPosition(7, 0);

      _usingPoseTuning = poseLayout.add("using pose tuning", false).withPosition(0, 0).getEntry();
      _xPoseError = poseLayout.add("x Pose Error", 0.0).withPosition(0, 1).getEntry();
      _yPoseError = poseLayout.add("y Pose Error", 0.0).withPosition(0, 2).getEntry();
      _thetaPoseError = poseLayout.add("theta Pose Error", 0.0).withPosition(0, 3).getEntry();
    }

    if (RobotAltModes.isPIDTuningMode) {
      int PIDCOL = 3;
      int PIDROW = 1;

      _errorRobotTheta = drivetrainTab.add("Error Angle", 0.0).getEntry();
      ShuffleboardLayout anglePIDLayout = drivetrainTab.getLayout("Angle PID", BuiltInLayouts.kGrid)
                                              .withSize(1, 2)
                                              .withPosition(PIDCOL, PIDROW);
      _anglePFac = anglePIDLayout.add("P", DRIVE_CONTROL.theta.p).withPosition(0, 0).getEntry();
      _angleIFac = anglePIDLayout.add("I", DRIVE_CONTROL.theta.i).withPosition(0, 1).getEntry();
      _angleDFac = anglePIDLayout.add("D", DRIVE_CONTROL.theta.d).withPosition(0, 2).getEntry();

      ShuffleboardLayout xLayout = drivetrainTab.getLayout("X PID", BuiltInLayouts.kGrid)
                                       .withSize(1, 2)
                                       .withPosition(PIDCOL + 1, PIDROW);
      _xPFac = xLayout.add("P", DRIVE_CONTROL.xy.p).withPosition(0, 0).getEntry();
      _xIFac = xLayout.add("I", DRIVE_CONTROL.xy.i).withPosition(0, 1).getEntry();
      _xDFac = xLayout.add("D", DRIVE_CONTROL.xy.d).withPosition(0, 2).getEntry();

      ShuffleboardLayout yLayout = drivetrainTab.getLayout("Y PID", BuiltInLayouts.kGrid)
                                       .withSize(1, 2)
                                       .withPosition(PIDCOL + 2, PIDROW);
      _yPFac = yLayout.add("P", DRIVE_CONTROL.xy.p).withPosition(0, 0).getEntry();
      _yIFac = yLayout.add("I", DRIVE_CONTROL.xy.i).withPosition(0, 1).getEntry();
      _yDFac = yLayout.add("D", DRIVE_CONTROL.xy.d).withPosition(0, 2).getEntry();
    }

    ShuffleboardTab controlboardTab = Shuffleboard.getTab("Competition HUD");
    controlboardTab.add("Field", _field).withSize(11, 5).withPosition(1, 1);
  }

  public void updateShuffleboard() {
    for (SwerveModule module : _modules) {
      module.updateShuffleboard();
    }

    _field.setRobotPose(_robotPoseEstimator.getEstimatedPosition());

    if (RobotAltModes.isPIDTuningMode) {
      _errorRobotTheta.setDouble(Units.degreesToRadians(_angleController.getPositionError()));
    }
  }

  public void simulationInit() {
    if (RobotAltModes.isSim) {
      for (SwerveModule module : _modules) module.simulationInit();
    }
  }

  public void simulationPeriodic() {
    if (RobotAltModes.isSim) {
      for (SwerveModule module : _modules) module.simulationPeriodic();

      _simPose = _simPose.transformBy(new Transform2d(
          new Translation2d(
              _chassisSpeeds.vxMetersPerSecond * Constants.LOOP_PERIOD_S,
              _chassisSpeeds.vyMetersPerSecond * Constants.LOOP_PERIOD_S
          ),
          Rotation2d.fromRadians(_chassisSpeeds.omegaRadiansPerSecond * Constants.LOOP_PERIOD_S)
      ));

      _gyro.getSimCollection().setRawHeading(_simPose.getRotation().getDegrees());
    }
  }

  public boolean isRedAlliance() {
    return _alliance == Alliance.Red;
  }

  public Rotation2d getYaw() {
    return Sensors.INVERT_GYRO ? Rotation2d.fromDegrees(360.0 - _gyro.getYaw())
                               : Rotation2d.fromDegrees(_gyro.getYaw());
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(_gyro.getPitch());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(_gyro.getRoll());
  }

  public Pose2d getPose() {
    return _robotPoseEstimator.getEstimatedPosition();
  }

  public double[] swerveMeasuredIO() {
    for (int i = 0; i < DRIVE_CONSTS.numModules; i++) {
      SwerveModuleState moduleState = _modules[i].getState();
      _entry[i * 2] = moduleState.angle.getDegrees();
      _entry[(i * 2) + 1] = moduleState.speedMetersPerSecond;
    }
    return _entry;
  }

  public double[] swerveSetpointsIO() {
    return _desiredSwerveStates;
  }

  public void setDesiredSwerveState(SwerveModuleState[] goalModuleStates) {
    for (int i = 0; i < DRIVE_CONSTS.numModules; i++) {
      SwerveModuleState state = goalModuleStates[i];
      _desiredSwerveStates[i * 2] = state.angle.getDegrees();
      _desiredSwerveStates[(i * 2) + 1] = state.speedMetersPerSecond;
    }
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return _robotPoseEstimator;
  }

  public edu.wpi.first.math.kinematics.SwerveDriveKinematics getSwerveKinematics() {
    return _swerveKinematics;
  }

  public boolean areWheelsAligned(SwerveModuleState[] goalStates) {
    for (int i = 0; i < DRIVE_CONSTS.numModules; i++) {
      if (!_modules[i].isAlignedTo(goalStates[i]))
        return false;
    }
    return true;
  }

  public boolean areWheelsAligned(SwerveModuleState goalState) {
    for (int i = 0; i < DRIVE_CONSTS.numModules; i++) {
      if (!_modules[i].isAlignedTo(goalState))
        return false;
    }
    return true;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule module : _modules) {
      module.resetToAbsolute();
    }
  }

  public void alignModulesToZero() {
    for (SwerveModule module : _modules) {
      module.alignToZero();
    }
  }

  public void updateAlliance(Alliance alliance) {
    _alliance = alliance;
  }

  public double POVZeroOffsetDeg() {
    return _alliance == Alliance.Red ? Constants.Sensors.POV_ZERO_RED_DEG
                                     : Constants.Sensors.POV_ZERO_BLUE_DEG;
  }

  public Rotation2d allianceGyroAngle() {
    return _alliance == Alliance.Red ? Constants.Sensors.GYRO_ZERO_RED
                                     : Constants.Sensors.GYRO_ZERO_BLUE;
  }

  public void zeroGyro() {
    setPose(_robotPoseEstimator.getEstimatedPosition(), allianceGyroAngle());
  }

  public void zeroPose() {
    setPose(new Pose2d());
  }

  public void autoZeroGyro() {
    setPose(
        _robotPoseEstimator.getEstimatedPosition(),
        getYaw().plus(Rotation2d.fromDegrees(POVZeroOffsetDeg()))
    );
  }

  public void setPose(Pose2d pose, Rotation2d yaw) {
    _gyro.setYaw(yaw.getDegrees());
    _robotPoseEstimator.resetPosition(
        yaw, _modulePositions, new Pose2d(pose.getTranslation(), yaw)
    );
  }

  public void setPose(Pose2d pose) {
    Rotation2d yaw = getYaw();
    _robotPoseEstimator.resetPosition(
        yaw, _modulePositions, new Pose2d(pose.getTranslation(), yaw)
    );
  }

  // TODO: implement
  public void setGyroAndPose(double yaw, Pose2d pose) {}

  public boolean motorResetConfig() {
    for (SwerveModule module : _modules) {
      if (module.motorResetConfig())
        return true;
    }
    return false;
  }

  public void setAutoPrepScore(boolean enable) {
    _autoPrepScore = enable;
  }

  public boolean getAutoPrepScore() {
    return _autoPrepScore;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates, double maxSpeed) {
    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DRIVE_CONSTS.model.theoreticalMaxWheelSpeed
    );

    for (SwerveModule module : _modules) {
      module.setDesiredState(desiredStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP);
    }

    setDesiredSwerveState(desiredStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, DRIVE_CONTROL.defaultLimits.maxTranslationalVelocityMPS);
  }

  public void drive(
      double joystickX,
      double joystickY,
      double joystickTheta,
      DriveModes mode,
      double maxSpeed_MPS,
      Rotation2d maxAngularSpeed
  ) {
    switch (mode) {
      case ROBOT_CENTRIC:
        robotCentricDrive(
            InputUtils.scaleJoystickX_MPS(joystickX, maxSpeed_MPS),
            InputUtils.scaleJoystickY_MPS(joystickY, maxSpeed_MPS),
            InputUtils.ScaleJoystickTheta_RADPS(joystickTheta, maxAngularSpeed)
        );
        break;
      case FIELD_RELATIVE:
        fieldRelativeDrive(
            InputUtils.scaleJoystickX_MPS(joystickX, maxSpeed_MPS),
            InputUtils.scaleJoystickY_MPS(joystickY, maxSpeed_MPS),
            InputUtils.ScaleJoystickTheta_RADPS(joystickTheta, maxAngularSpeed)

        );
        break;
      case SNAP_TO_ANGLE:
        snapToAngleDrive(
            InputUtils.scaleJoystickX_MPS(joystickX, maxSpeed_MPS),
            InputUtils.scaleJoystickY_MPS(joystickY, maxSpeed_MPS)
        );
        break;
      default:
        // ERROR
        // throw, error message, or default behavior
        // return;
        break;
    }
  }

  public void drive(double joystickX, double joystickY, double joystickTheta) {
    drive(joystickX, joystickY, joystickTheta, DriveModes.FIELD_RELATIVE);
  }

  public void drive(double joystickX, double joystickY, double joystickTheta, DriveModes mode) {
    drive(
        joystickX,
        joystickY,
        joystickTheta,
        mode,
        DRIVE_CONTROL.defaultLimits.maxTranslationalVelocityMPS
    );
  }

  public void drive(
      double joystickX, double joystickY, double joystickTheta, DriveModes mode, double maxSpeed_MPS
  ) {
    drive(
        joystickX,
        joystickY,
        joystickTheta,
        mode,
        maxSpeed_MPS,
        DRIVE_CONTROL.defaultLimits.maxAngularVelocityPS
    );
  }

  // individual drive functions
  public void robotCentricDrive(
      double translationX_MPS, double translationY_MPS, double rotation_RADPS, double maxSpeed
  ) {
    SwerveModuleState[] goalModuleStates =
        _swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            translationX_MPS, translationY_MPS, rotation_RADPS, getYaw()
        ));

    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        goalModuleStates, DRIVE_CONSTS.model.theoreticalMaxWheelSpeed
    );

    for (SwerveModule module : _modules) {
      module.setDesiredState(
          goalModuleStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP
      );
    }
    setDesiredSwerveState(goalModuleStates);
  }

  public void robotCentricDrive(
      double translationX_MPS, double translationY_MPS, double rotation_RADPS
  ) {
    robotCentricDrive(
        translationX_MPS,
        translationY_MPS,
        rotation_RADPS,
        DRIVE_CONTROL.defaultLimits.maxTranslationalVelocityMPS
    );
  }

  public boolean isOversaturated(SwerveModuleState[] states) {
    for (SwerveModuleState state : states) {
      if (state.speedMetersPerSecond > DRIVE_CONSTS.model.theoreticalMaxWheelSpeed)
        return true;
    }
    return false;
  }

  public void fieldRelativeDrive(
      double translationX_MPS, double translationY_MPS, double rotation_RADPS
  ) {
    SwerveModuleState[] goalModuleStates =
        _swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            translationX_MPS,
            translationY_MPS,
            rotation_RADPS,
            USE_POSE_ESTIMATION_ANGLE ? _robotPoseEstimator.getEstimatedPosition().getRotation()
                                      : getYaw()
        ));

    // Desaturate based of max theoretical or functional rather than current max
    edu.wpi.first.math.kinematics.SwerveDriveKinematics.desaturateWheelSpeeds(
        goalModuleStates, DRIVE_CONSTS.model.theoreticalMaxWheelSpeed
    );
    for (SwerveModule module : _modules) {
      module.setDesiredState(
          goalModuleStates[module._moduleNumber], Constants.Control.IS_OPEN_LOOP
      );
    }
    //    _desiredSpeed.setDouble(goalModuleStates[0].speedMetersPerSecond);
    setDesiredSwerveState(goalModuleStates);
  }

  public void slewingFieldRelativeDrive(
      double translationX_MPS, double translationY_MPS, double rotation_RADPS
  ) {
    fieldRelativeDrive(
        _xSlewRateFilter.calculate(translationX_MPS),
        _ySlewRateFilter.calculate(translationY_MPS),
        _angleSlewRateFilter.calculate(rotation_RADPS)
    );
  }

  public void snapToAngleDrive(
      double translationX_MPS, double translationY_MPS, double maxSpeed_MPS
  ) {
    fieldRelativeDrive(
        translationX_MPS,
        translationY_MPS,
        // calculate snap to angle rotation here
        // TODO: reconfigure deadband logic to limit jittering
        _angleController.atGoal() ? 0.0 : _angleController.calculate(getYaw().getRadians())
    );
  }

  public void snapToAngleDrive(double translationX_MPS, double translationY_MPS) {
    snapToAngleDrive(
        translationX_MPS, translationY_MPS, DRIVE_CONTROL.defaultLimits.maxTranslationalVelocityMPS
    );
  }

  public void setTolerance(double xTolerance, double yTolerance, Rotation2d thetaTolerance) {
    _xController.setTolerance(xTolerance);
    _yController.setTolerance(yTolerance);
    _angleController.setTolerance(thetaTolerance.getRadians());
  }

  public void resetDefaultTolerance() {
    setTolerance(
        DRIVE_CONTROL.xy.toleranceM, DRIVE_CONTROL.xy.toleranceM, DRIVE_CONTROL.theta.tolerance
    );
  }

  public void chaseStaticTargetDrive(double maxSpeed_MPS) {
    Pose2d currentPose = getPose();

    if (RobotAltModes.isUnprofiledPIDMode) {
      // TODO Test/replace with slewing drive (?)
      fieldRelativeDrive(
          _xController.atSetpoint() ? 0.0 : _xController.calculate(currentPose.getX()),
          _yController.atSetpoint() ? 0.0 : _yController.calculate(currentPose.getY()),
          _angleController.atSetpoint() ? 0.0 : _angleController.calculate(getYaw().getRadians())
      );
    } else {
      // Profiled controllers naturally filter/slew
      if (RobotAltModes.isPoseTuning) {
        _xPoseError.setDouble(_xController.getPositionError());
        _yPoseError.setDouble(_yController.getPositionError());
        _thetaPoseError.setDouble(Units.radiansToDegrees(_angleController.getPositionError()));
      }

      fieldRelativeDrive(
          _xController.calculate(currentPose.getX()),
          _yController.calculate(currentPose.getY()),
          _angleController.calculate(getYaw().getRadians())
      );
    }
  }

  public void chaseStaticTargetDrive() {
    chaseStaticTargetDrive(DRIVE_CONTROL.defaultLimits.maxTranslationalVelocityMPS);
  }

  public void setSnapAngleDeg(double angle_DEG) {
    setSnapAngleRad(Units.degreesToRadians(angle_DEG));
  }

  public void setSnapAngle(Rotation2d angle) {
    setSnapAngleRad(angle.getRadians());
  }

  public void setSnapAngleRad(double angle_rad) {
    _angleController.reset(
        getPose().getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
    );
    _angleController.setGoal(angle_rad);
  }

  public void setStaticTarget(Pose2d goalPose) {
    Pose2d currentPose = getPose();
    _xController.reset(currentPose.getX(), _chassisSpeeds.vxMetersPerSecond);
    _yController.reset(currentPose.getY(), _chassisSpeeds.vyMetersPerSecond);
    _angleController.reset(
        currentPose.getRotation().getRadians(), _chassisSpeeds.omegaRadiansPerSecond
    );
    _xController.setGoal(goalPose.getX());
    _yController.setGoal(goalPose.getY());
    _angleController.setGoal(goalPose.getRotation().getRadians());
  }

  public Rotation2d toAbsoluteAngle(Rotation2d angle) {
    return toAbsoluteAngle(angle);
  }

  public double toAbsoluteAngleRad(double angle_rad) {
    return Units.degreesToRadians(toAbsoluteAngleDeg(Units.radiansToDegrees(angle_rad)));
  }

  public double toAbsoluteAngleDeg(double angle_deg) {
    double scaled = (angle_deg % 360.0);
    return scaled + (scaled < 0 ? 360.0 : 0.0);
  }

  public void testSteer(double angle_deg) {
    for (var module : _modules) module.setLastAngleDeg(angle_deg);
  }

  public void autonomousDriveMode(boolean enable) {
    for (SwerveModule module : _modules) {
      module.autonomousDriveMode(enable);
    }
  }

  public boolean inRange() {
    return (_xController.atGoal() && _yController.atGoal() && _angleController.atGoal());
  }

  public boolean thetaInRange() {
    return _angleController.atGoal();
  }

  public Pose2d getAlliancePose(Pose2d red, Pose2d blue) {
    return _alliance == Alliance.Red ? red : blue;
  }

  public void updateTuneScoringShuffleboard(double xError, double yError, double thetaError) {
    if (RobotAltModes.isPoseTuning) {
      _xPoseError.setDouble(xError);
      _yPoseError.setDouble(yError);
      _thetaPoseError.setDouble(thetaError);
    }
  }

  public void updateTuneScoringStatus(boolean tuneScoring) {
    if (RobotAltModes.isPoseTuning) {
      _usingPoseTuning.setBoolean(tuneScoring);
    }
  }

  public InstantCommand zeroGyroCommand() {
    return new InstantCommand(() -> zeroGyro());
  }

  public InstantCommand zeroPoseCommand() {
    return new InstantCommand(() -> zeroPose());
  }

  private boolean checkInitStatus() {
    ErrorCode initStatus = _gyro.configFactoryDefault();
    return (initStatus == ErrorCode.OK);
  }
}
