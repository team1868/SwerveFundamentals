// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.base.GoToPoseCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotAltModes;
import frc.robot.constants.enums.AutoCommand;
import frc.robot.constants.enums.AutonomousAction;
import frc.robot.constants.enums.AutonomousRoutines;
import frc.robot.constants.enums.AutonomousRoutines.Builders;
import frc.robot.constants.enums.DriveModes;
import frc.robot.subsystems.*;
import java.util.Map;

public class RobotContainer {
  /* --- Shared Resources --- */
  private Field2d _field = new Field2d();

  /* --- Subsystems --- */
  public Controlboard _controlboard = new Controlboard(_field);
  public Drivetrain _drivetrain = new Drivetrain(_field);
  public LedController _ledController = new LedController(20);

  /* --- Commands --- */
  // Drivetrain
  private TeleopSwerveCommand _teleopSwerveCommand =
      new TeleopSwerveCommand(_drivetrain, _controlboard, DriveModes.FIELD_RELATIVE);
  private InstantCommand _zeroGyroCommand = _drivetrain.zeroGyroCommand();
  private InstantCommand _zeroPoseCommand = _drivetrain.zeroPoseCommand();

  // LED
  private SolidLedCommand _teleopLEDCommand =
      new SolidLedCommand(_ledController, LedController.DEFAULT_COLOR);

  // Go to pose testing
  private GoToPoseCommand _goToPoseCommandPos1 =
      new GoToPoseCommand(_drivetrain, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
  private GoToPoseCommand _goToPoseCommandPos2 =
      new GoToPoseCommand(_drivetrain, new Pose2d(-2.0, -2.0, Rotation2d.fromDegrees(0.0)));

  /* Shuffleboard */
  // Auto and Path Planner
  private Alliance _alliance = Alliance.Invalid;
  private SendableChooser<Integer> _autoChooser = new SendableChooser<Integer>();
  public AutonomousRoutines _curAutoSelected = AutonomousRoutines.DEFAULT_AUTO;

  private Map<String, Command> _eventMap;
  private AutonomousRoutines[] _autonModes;

  public RobotContainer() {
    var setupTab = Shuffleboard.getTab("Setup");

    // Auto Chooser
    setupTab.add("Auto Chooser", _autoChooser).withSize(3, 2);
    _autoChooser.setDefaultOption("NO AUTONOMOUS", AutonomousRoutines.DEFAULT_AUTO.ordinal());

    // Default commands
    _drivetrain.setDefaultCommand(_teleopSwerveCommand);
    _ledController.setDefaultCommand(_teleopLEDCommand);

    registerAutonomousCommands();
    registerAutonomousRoutines();

    configureBindings();
    configShuffleboard();
  }

  private void registerAutonomousUtilityCommands() {
    /* --- Print marker commands --- */
    // Use these to log data
    AutonomousAction.registerAutonomousAction("printStart", "Start Auto Sequence");

    /* --- Wait Commands --- */
    // Use these for parallel deadline stop points to stop the drivetrain from moving
    AutonomousAction.registerAutonomousAction("waitScore", 1.75);

    /* --- LED Commands --- */
    AutonomousAction.registerAutonomousAction("orangeLed", _ledController, Color.kOrange);
    AutonomousAction.registerAutonomousAction("yellowLed", _ledController, Color.kYellow);
    AutonomousAction.registerAutonomousAction("pinkLed", _ledController, Color.kPink);
    AutonomousAction.registerAutonomousAction("greenLed", _ledController, Color.kGreen);
    AutonomousAction.registerAutonomousAction("blueLed", _ledController, Color.kBlue);
    AutonomousAction.registerAutonomousAction("purpleLed", _ledController, Color.kPurple);
  }

  private void registerAutonomousCommands() {
    registerAutonomousUtilityCommands();

    _eventMap = AutonomousAction.getEventMap();
  }

  private void registerAutonomousRoutines() {
    if (_autonModes == null) {
      for (Builders builder : Builders.values()) {
        builder.initialize(_drivetrain, _eventMap);
      }

      _autonModes = AutonomousRoutines.values();
      for (AutonomousRoutines routine : _autonModes) {
        if (routine.showInDashboard) {
          routine.build(_drivetrain, _controlboard);
          _autoChooser.addOption(routine.name, routine.ordinal());
        }
      }
    }
  }

  public AutonomousRoutines getAutonomousRoutineSelection() {
    var result = _autoChooser.getSelected();
    return result == null ? null : _autonModes[result.intValue()];
  }

  public void configFMSData() {
    _alliance = DriverStation.getAlliance();
    _drivetrain.updateAlliance(_alliance);
    _controlboard.updateAlliance(_alliance);
  }

  public void configShuffleboard() {}

  public void robotInit() {
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
  }

  public void autonomousInit() {
    _drivetrain.enableAuto();
    _drivetrain.resetModulesToAbsolute();
    _drivetrain.autonomousDriveMode(true);
  }

  public void teleopInit() {
    _drivetrain.disableAuto();
    _drivetrain.resetModulesToAbsolute();
    _drivetrain.autonomousDriveMode(false);
    _teleopSwerveCommand.teleopInit();
  }

  public void simulationInit() {
    _drivetrain.simulationInit();
  }

  public void periodic() {
    _controlboard.updateShuffleboard();
  }

  public void disabledPeriodic() {
    AutonomousRoutines prev = _curAutoSelected;
    _curAutoSelected = getAutonomousRoutineSelection();

    Alliance prevAlliance = _alliance;
    _alliance = DriverStation.getAlliance();

    if (_curAutoSelected == AutonomousRoutines.DEFAULT_AUTO) {
      // No auto selected, turn entire strip red
      _ledController.setSolidColor(Color.kRed);
    } else {
      // For initial pose aligning
      if (prev != _curAutoSelected || prevAlliance != _alliance) {
        _drivetrain.updateAlliance(_alliance);
        _controlboard.updateAlliance(_alliance);

        _ledController.setSolidColor(new Color(0, 0, 0));
      }
    }
  }

  public void onEnable() {
    _drivetrain.onEnable();
  }

  public void onDisable() {
    resetAll();
    _controlboard.driverResetRumble();
  }

  public void resetAll() {}

  private void configureBindings() {
    // DRIVER
    _controlboard._xboxDrive.x().onTrue(_zeroGyroCommand);
    _controlboard._xboxDrive.start().onTrue(_zeroPoseCommand);
  }
}
