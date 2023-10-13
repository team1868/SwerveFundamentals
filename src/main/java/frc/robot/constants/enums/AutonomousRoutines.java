package frc.robot.constants.enums;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Controlboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PIDConstants;
import java.util.Map;
import java.util.Vector;

public enum AutonomousRoutines {
  DEFAULT_AUTO(true, "DEFAULT", Commands.print("DEFAULT AUTO SAYS HI"));

  public final boolean showInDashboard;
  public final String name;
  public final boolean buildable;
  public final Builders builder;
  public final boolean balanceAtEnd;
  public final boolean yeetCone;

  private AutonomousPathLegs[] _paths;
  private PathPlannerState initialState;
  private Pose2d initialRedPose, initialBluePose;
  public CommandBase command;
  public CommandBase builtCommand;

  AutonomousRoutines(String shuffleboardName, CommandBase command) {
    this(true, shuffleboardName, command);
  }
  AutonomousRoutines(boolean show, String shuffleboardName, CommandBase command) {
    this.showInDashboard = show;
    this.name = shuffleboardName;
    this.command = command;

    this.buildable = false;
    this.builder = null;
    this.balanceAtEnd = false;
    this.yeetCone = false;
  }

  AutonomousRoutines(String shuffleboardName, AutonomousPathLegs[] paths) {
    this(true, shuffleboardName, paths);
  }

  AutonomousRoutines(boolean show, String shuffleboardName, AutonomousPathLegs[] paths) {
    this(show, shuffleboardName, paths, Builders.DEFAULT_BUILDER);
  }

  AutonomousRoutines(String shuffleboardName, AutonomousPathLegs[] paths, Builders builder) {
    this(true, shuffleboardName, paths, builder);
  }

  AutonomousRoutines(
      boolean show, String shuffleboardName, AutonomousPathLegs[] paths, Builders builder
  ) {
    this(show, shuffleboardName, paths, builder, false, false);
  }

  AutonomousRoutines(
      boolean show,
      String shuffleboardName,
      AutonomousPathLegs[] paths,
      Builders builder,
      boolean balance,
      boolean yeet
  ) {
    this.showInDashboard = show;
    this.name = shuffleboardName;

    this.buildable = builder != null && paths.length > 0;
    this.builder = builder;
    this._paths = paths;

    this.balanceAtEnd = balance;
    this.yeetCone = yeet;
  }

  public void build(Drivetrain drivetrain, Controlboard controlboard) {
    if (buildable) {
      Vector<PathPlannerTrajectory> pathSet = new Vector<PathPlannerTrajectory>();
      for (AutonomousPathLegs p : _paths) {
        if (p.currentlyExists) {
          pathSet.add(PathPlanner.loadPath(p.name, p.constraint.pathPlanner));
        } else {
          System.out.println("Auto routine with currently undefined path " + p.name);
          System.exit(1);
        }
      }
      initialState = pathSet.get(0).getInitialState();
      Pose2d initialPose = initialState.poseMeters;
      initialBluePose =
          new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(180.0));
      initialRedPose = new Pose2d(
          Constants.CField.dims.x_M - initialPose.getX(),
          initialPose.getY(),
          Rotation2d.fromDegrees(0.0)
      );

      builtCommand = builder.construct(pathSet)
                         .finallyDo((boolean interrupted) -> drivetrain.autoZeroGyro())
                         .beforeStarting(
                             ()
                                 -> {
                                     // any initial configuration
                                 }
                         );

      // The order of these matters, a lot
      if (balanceAtEnd && yeetCone) {
        command = Commands.sequence(builtCommand);
      } else if (balanceAtEnd) {
        command = Commands.sequence(builtCommand);
      } else if (yeetCone) {
        command = Commands.sequence(builtCommand);
      } else {
        command = builtCommand;
      }
    }
  }

  public void configureCommand(CommandBase command) {
    this.command = command;
  }

  public Pose2d getInitialPose(Alliance alliance) {
    return alliance == Alliance.Blue ? initialBluePose : initialRedPose;
  }

  public enum Builders {
    DEFAULT_BUILDER(Constants.CRobot.drive.control.xy, Constants.CRobot.drive.control.theta),
    AUTO_BUILDER(Constants.CRobot.drive.control.xy, Constants.CRobot.drive.control.autonTheta),
    CHARGING_BUILDER(
        Constants.CRobot.drive.control.xy, Constants.CRobot.drive.control.chargerAutonTheta
    );

    private SwerveAutoBuilder builder;
    private final PIDConstants xy;
    private final PIDConstants theta;

    Builders(PIDConstants xy, PIDConstants theta) {
      this.xy = xy;
      this.theta = theta;
    }

    public void initialize(Drivetrain drivetrain, Map<String, Command> eventMap) {
      builder = new SwerveAutoBuilder(
          ()
              -> { return drivetrain.getPose(); },
          (Pose2d initPose)
              -> { drivetrain.setPose(initPose, initPose.getRotation()); },
          drivetrain.getSwerveKinematics(),
          xy.getPathPlannerTranslation(),
          theta.getPathPlannerTheta(),
          (SwerveModuleState[] states)
              -> { drivetrain.setModuleStates(states); },
          eventMap,
          true,
          drivetrain
      );
    }

    public CommandBase construct(Vector<PathPlannerTrajectory> trajectories) {
      return builder.fullAuto(trajectories);
    }
  }

  public enum PPPConstraints {
    AUTO_DEFAULT(3.4, 1.8),
    CHARGE(2.0, 2.0),
    CHARGE_TRAVERSAL(1.4, 1.6);

    public final com.pathplanner.lib.PathConstraints pathPlanner;

    PPPConstraints(double maxVelo_mps, double maxAccel_mps2) {
      pathPlanner = new com.pathplanner.lib.PathConstraints(maxVelo_mps, maxAccel_mps2);
    }
  }
}
