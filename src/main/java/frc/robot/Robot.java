package frc.robot;

import java.util.ArrayList;

import org.ietf.jgss.Oid;
import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.Commands.DomainExpansion.AutoAllignL;
import frc.robot.Commands.DomainExpansion.AutoAllignR;
import frc.robot.Commands.DomainExpansion.GroundAlgaeIntake;
import frc.robot.Commands.DomainExpansion.GroundIntakeCoral;
import frc.robot.Commands.DomainExpansion.HighAlgaeIntake;
import frc.robot.Commands.DomainExpansion.HumanPlayerIntake;
import frc.robot.Commands.DomainExpansion.Intake;
import frc.robot.Commands.DomainExpansion.L1CoralScore;
import frc.robot.Commands.DomainExpansion.L2CoralScore;
import frc.robot.Commands.DomainExpansion.L3CoralScore;
import frc.robot.Commands.DomainExpansion.L4CoralScore;
import frc.robot.Commands.DomainExpansion.LowAlgaeIntake;
import frc.robot.Commands.DomainExpansion.Outtake;
import frc.robot.Commands.DomainExpansion.ProcessorScore;
import frc.robot.Commands.DomainExpansion.Transition;
import frc.robot.Commands.DomainExpansion.TransitionAuto;
import frc.robot.Commands.AlignToCoral;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.subsystems.claw.Wrist;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
//import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.MAXSwerveModule;
import frc.robot.subsystems.swerve.DriveSubsystem.DriveState;
import frc.robot.subsystems.vision.CameraSystem;
import frc.robot.subsystems.vision.CameraSystem.PoleSide;

public class Robot extends LoggedRobot {
  // all the initialing for the systems
  public double speedMod;
  public double clawZeroPower;
  private Climber climber;
  private DriveSubsystem drivebase;
  private Wrist wrist;
  private Claw claw;
  private Pivot pivot; 
  private Elevator elevator;
  private CameraSystem camSystem;
  private WristCommand wristCommand;
  private GroundAlgaeIntake groundAlgaeIntake;
  private GroundIntakeCoral groundCoralIntake;
  private HighAlgaeIntake highAlgaeIntake;
  private HumanPlayerIntake humanPlayerIntake;
  private L1CoralScore L1CoralScore;
  private L2CoralScore L2CoralScore;
  private L3CoralScore L3CoralScore;
  private L4CoralScore L4CoralScore;
  private LowAlgaeIntake lowAlgaeIntake;
  private ProcessorScore processorScore;
  private PivotCommand pivotCommand;
  private ElevatorCommand elevatorCommand;
  private Transition transition;
  private TransitionAuto transitionAuto;
  private Outtake outtake;
  private Intake intake;
  private AutoAllignR autoAllignR;
  private AutoAllignL autoAllignL;
  private AlignToCoral alignToCoral;
  // private LED litty;
  // private CameraSystem camSystem;
  private String mode = "coral";
  public double rumbleTimer;
  public double switchTimer;
  public double transTimer;
  public RobotConfig Rconfig;
  private static XboxController driver;
  private static XboxController operator;
  // initialization of the auton chooser in the dashboard
  private Command m_autoSelected;
  Double targetRange = null;
  Double targetAngle = null;
  double invert = 1;
  
  // that is a chooser for the autons utilizing the sendableChooser which allows
  // us to choose the auton commands
  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  @Override
  public void robotInit() {
    
    speedMod = 1;
    drivebase = DriveSubsystem.getInstance();
    elevator = Elevator.getInstance();
    // litty = LED.getInstance();
    wrist = Wrist.getInstance();
    climber = Climber.getInstance();
    claw = Claw.getInstance();
    pivot = Pivot.getInstance();
    camSystem = CameraSystem.getInstance(); 
    camSystem.AddCamera(new PhotonCamera("ClimbCam"), new Transform3d(
      new Translation3d(-0.30043, -0.26457, 0.31945), new Rotation3d(0.0, 0.0, Math.toRadians(-55.56095))), 
      true);
    camSystem.AddCamera(new PhotonCamera("SwerveCam"), new Transform3d(
      new Translation3d(0.28831, -0.2421, 0.29561), new Rotation3d(0.0, Math.toRadians(2.5), Math.toRadians(-105))), 
      true);
      camSystem.AddCamera(new PhotonCamera("MiddleCam"), new Transform3d(
        new Translation3d(0.00833, -0.22138, 0.14534), new Rotation3d(0.0, 0.0, Math.toRadians(-90))), 
        true);
    transitionAuto = new TransitionAuto();
    wristCommand = new WristCommand(WristState.TEST);
    pivotCommand = new PivotCommand(PivotState.TRANSITIONSTATE);
    elevatorCommand = new ElevatorCommand(ElevatorState.TEST);
    groundAlgaeIntake = new GroundAlgaeIntake();
    highAlgaeIntake = new HighAlgaeIntake();
    humanPlayerIntake = new HumanPlayerIntake();
    L1CoralScore = new L1CoralScore();
    L2CoralScore = new L2CoralScore();
    L3CoralScore = new L3CoralScore();
    L4CoralScore = new L4CoralScore();
    lowAlgaeIntake = new LowAlgaeIntake();
    processorScore = new ProcessorScore();
    transition = new Transition();
    intake = new Intake();
    outtake = new Outtake();
    autoAllignR = new AutoAllignR();
    autoAllignL = new AutoAllignL();
    
    alignToCoral = new AlignToCoral();
    // camSystem = CameraSystem.getInstance();
    // camSystem.AddCamera(new PhotonCamera("Cam1"), new Transform3d(
    // new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0))
    // , true);

    // camSystem.AddCamera(new PhotonCamera("Cam2"), new Transform3d(
    // new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
    // true);

    driver = new XboxController(0);
    operator = new XboxController(1);
    
    NamedCommands.registerCommand("L4Score", L4CoralScore);
    NamedCommands.registerCommand("HumanIntake", humanPlayerIntake);
    NamedCommands.registerCommand("L3Score", L3CoralScore);
    NamedCommands.registerCommand("HighAlgae", highAlgaeIntake);
    NamedCommands.registerCommand("LowAlgae", lowAlgaeIntake);
    NamedCommands.registerCommand("ProcessorScore", processorScore);
    NamedCommands.registerCommand("Elevator", elevatorCommand);
    NamedCommands.registerCommand("Transition", transition);
    NamedCommands.registerCommand("outtake", outtake);
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("AutoAllignR", autoAllignR);
    NamedCommands.registerCommand("AutoAllignL", autoAllignL);
    m_chooser.addOption("1_C_1_P1C", new PathPlannerAuto("1_C_1_P1C"));
    m_chooser.addOption("test", new PathPlannerAuto("test"));
    m_chooser.addOption("test2", new PathPlannerAuto("test2"));
    m_chooser.addOption("straight", new PathPlannerAuto("straight"));
    // SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData(m_chooser);
    
  }

  @Override
  public void robotPeriodic() {
    
    //camSystem.isBlue = SmartDashboard.getBoolean("isBlue", camSystem.isBlue);
    SmartDashboard.putNumber("MotorL Current", wrist.MotorL.getOutputCurrent());
    SmartDashboard.putNumber("feedforwards/feedforwardL", wrist.feedforwardL.calculate(2*Math.PI*wrist.MotorL.getAbsoluteEncoder().getPosition(), 0));
    SmartDashboard.putNumber("feedforwards/feedforwardR", wrist.feedforwardR.calculate(2*Math.PI*wrist.MotorL.getAbsoluteEncoder().getPosition(), 0));
    SmartDashboard.putNumber("Left abs encoder", wrist.MotorL.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Right abs encoder", wrist.MotorR.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("Reefscape/DiffWrist/pitch Encoder Pos", wrist.getPitchAngle());
    SmartDashboard.putNumber("Reefscape/DiffWrist/roll Encoder Pos", wrist.getRollAngle());
    SmartDashboard.putNumber("Reefscape/DiffWrist/pitchPID Setpoint",wrist. pitchPID.getSetpoint());
    SmartDashboard.putNumber("Reefscape/DiffWrist/rollPID Setpoint", wrist.rollPID.getSetpoint());
    SmartDashboard.putNumber("Left voltage", wrist.getVoltageLeft());
    SmartDashboard.putNumber("Right voltage", wrist.getVoltageRight());
    SmartDashboard.putNumber("Pitch voltage", wrist.pitchPID.calculate(wrist.pitchEncoder.getPosition()));
    SmartDashboard.putNumber("Roll voltage", wrist.rollPID.calculate(wrist.rollEncoder.getPosition()));

    SmartDashboard.putNumber("Pivot Encoder", pivot.pivotMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putString("Pivot Desired Pos", pivot.getState().toString());
    SmartDashboard.putString("Elevator Desired Pos", elevator.getState().toString());


    SmartDashboard.putString("intakemode", mode);


    SmartDashboard.putNumber("pivot feedforward", pivot.feedForward.calculate(Math.toRadians(pivot.pivotMotor.getAbsoluteEncoder().getPosition()),0));

    SmartDashboard.putNumber("elevator feedforward", elevator.feedForward.calculate(elevator.elevatorMotorL.getEncoder().getPosition(), 0));
    SmartDashboard.putNumber("elevator encoder", elevator.getElevatorPosition());
    SmartDashboard.putBoolean("elevator Top Switch", elevator.getTopLimitSwitch());
    SmartDashboard.putBoolean("elevator Bot Switch", elevator.getBottomLimitSwitch());
    SmartDashboard.putBoolean("algae Breakbeam", claw.getAlgaeBreakBeam());
    SmartDashboard.putBoolean("coral Breakbeam", claw.getCoralBreakBeam());
    SmartDashboard.putString("wristlaststate", wrist.lastState.toString());
    SmartDashboard.putBoolean("pivotisfinished", pivotCommand.isFinished());

    SmartDashboard.putNumber("Last Tag Seen", camSystem.lastTag);
    SmartDashboard.putNumber("Desired Degree", 
    CameraSystem.aprilTagFieldLayout.getTagPose(18).get().getRotation().toRotation2d().getDegrees());
    SmartDashboard.putNumber("Currenr Degree", DriveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    if(camSystem.getTargetRange(1, camSystem.lastTag) != null && camSystem.getYawForTag(1, camSystem.lastTag)!= null)
    {
      SmartDashboard.putNumber("SwerveCam dist", camSystem.getTargetRange(1, camSystem.lastTag));
      SmartDashboard.putNumber("SwerveCam yaw", camSystem.getYawForTag(1, camSystem.lastTag));
    }
    if(camSystem.getTargetRange(0, camSystem.lastTag) != null && camSystem.getYawForTag(0, camSystem.lastTag)!= null)
    {
      SmartDashboard.putNumber("ClimbCam dist", camSystem.getTargetRange(0, camSystem.lastTag));
      SmartDashboard.putNumber("ClimbCam yaw", camSystem.getYawForTag(0, camSystem.lastTag));
    }
    SmartDashboard.putNumber("heading", drivebase.getWorkingHeading());
    if(alignToCoral != null && alignToCoral.xController != null 
    && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null
    && alignToCoral.yController != null && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null
    && alignToCoral.thetaController != null){
      SmartDashboard.putNumber("xController", alignToCoral.xController.calculate(
        camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()));
      SmartDashboard.putNumber("yController", alignToCoral.yController.calculate(
        camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()));
      SmartDashboard.putNumber("thetaController", alignToCoral.thetaController.calculate(drivebase.getWorkingHeading()));
    }
    
    SmartDashboard.putString("Focus Cam", camSystem.focusCamIndex == 0 ? "climb" : "swerve");
    // SmartDashboard.putNumber("Currenr Degree", CameraSystem.get);


    

    // SmartDashboard.putNumber("currentPose Angle L", wrist.ThoseWhoTroll[0]);
    // SmartDashboard.putNumber("currentPose Angle R", wrist.ThoseWhoTroll2[0]);
    // SmartDashboard.putNumber("currentPose section L", wrist.ThoseWhoTroll[1]);
    // SmartDashboard.putNumber("currentPose section R", wrist.ThoseWhoTroll2[1]);

    // SmartDashboard.putNumber("1st Angle L", wrist.WrapAngle(wrist.currentPositionL, wrist.wristState.tiltPosition)[0]);
    // SmartDashboard.putNumber("2st Angle L", wrist.WrapAngle(wrist.currentPositionL, wrist.wristState.tiltPosition)[1]);
    // SmartDashboard.putNumber("1st Angle R", wrist.WrapAngle(wrist.currentPositionL, wrist.wristState.tiltPosition)[0]);
    // SmartDashboard.putNumber("2st Angle R", wrist.WrapAngle(wrist.currentPositionL, wrist.wristState.tiltPosition)[1]);

    // SmartDashboard.putString("Wrist State", wrist.wristState.toString());

    
   
    
    // Pose2d cameraPositionTele = camSystem.calculateRobotPosition();

    // Pose2d posTele = drivebase.updateOdometry(cameraPositionTele);
   
    // SmartDashboard.putNumber("Odometry X", posTele.getX());
    // SmartDashboard.putNumber("Odometry Y", posTele.getY());

    // this is getting the data from the cameras through the cameraSystem class
    // if (camSystem.getCamera(0).isConnected()) {
    // PhotonPipelineResult backResult = camSystem.getResult(0);
    // if (backResult.hasTargets()) {
    // PhotonTrackedTarget target = backResult.getBestTarget();
    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    // double distance = bestCameraToTarget.getTranslation().getNorm();
    // SmartDashboard.putString("Back Camera Target", "Yes Targets");
    // SmartDashboard.putNumber("Back to Target", distance);
    // SmartDashboard.putNumber("Back Camera Target Yaw", target.getYaw());
    // SmartDashboard.putNumber("Back Camera Target Pitch", target.getPitch());
    // SmartDashboard.putNumber("Back Camera Target Area", target.getArea());
    // SmartDashboard.putNumber("ID", target.getFiducialId());

    // } else if(backResult.hasTargets() == false) {
    // SmartDashboard.putString("Back Camera Target", "No Targets");
    // }
    // }
    // testing the valuies that the camera gives us and outputing it into the
    // dashboard
    // Pose2d cameraPosition = camSystem.calculateRobotPosition();
    // SmartDashboard.putNumber("Camera X Position", cameraPosition.getX());
    // SmartDashboard.putNumber("Camera Y Position", cameraPosition.getY());
    // SmartDashboard.putNumber("Camera Heading",
    // cameraPosition.getRotation().getDegrees());

    CommandScheduler.getInstance().run();
    drivebase.periodic();

    wrist.periodic();


    // putting all of the info from the subsystems into the dashvoard so we can test
    // things
    
    SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

  }

  @Override
  public void autonomousInit() {
    robotInit();
    // getting the value we chose from the dashboard and putting it into motion in
    // the auton
   
    m_autoSelected = m_chooser.getSelected();

     //drivebase.resetOdometry(PathPlannerAuto.getStartingPoseFromAutoFile(m_chooser.getSelected().getName()));

    // schedules the command so it actually begins moving
    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // updating the intake for the autointake command
    // using cameras to calculate the robot position instead of odometry.
    // we use a mix of odometry + camera positions to calculate the robot position
    // Pose2d cameraPosition = camSystem.calculateRobotPosition();
    // Pose2d pose = drivebase.updateOdometry(cameraPosition);

    // SmartDashboard.putNumber("Auto X", drivebase.getPose().getX());
    // SmartDashboard.putNumber("Auto Y", drivebase.getPose().getY());
    // SmartDashboard.putNumber("Odometry X", pose.getX());
    // SmartDashboard.putNumber("Odometry Y", pose.getY());
    drivebase.periodic();
    camSystem.updateLatestResult(false);
  }

  @Override
  public void teleopInit() {
    robotInit();
    // as soon as we begin teleop we desable the auton selection
    // litty.setBlue();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }

    Translation2d testxy = new Translation2d(16.57 - 14.7, 5.54);
    Rotation2d testRot = new Rotation2d(0);
    Pose2d test = new Pose2d(testxy, testRot);
    // drivebase.resetOdometry(test);
  }

  @Override
  public void teleopPeriodic() {
    elevator.updatePose();
    pivot.updatePose();
    
    double ySpeed = drivebase.inputDeadband(-driver.getLeftX()) * speedMod;
    double xSpeed = drivebase.inputDeadband(-driver.getLeftY()) * speedMod;
    double rot = drivebase.inputDeadband(-driver.getRightX()) * speedMod;
    if(driver.getRightTriggerAxis() >= .5){
      speedMod = .5;
    }else if(driver.getLeftTriggerAxis() >= .5){
      speedMod = .1;
    }else{
      speedMod = 1;
    }
    camSystem.updateLatestResult(driver.getBButton() || driver.getXButton());
    // if(pivot.getPosition() >= 120){
    //   ySpeed *= .7;
    //   xSpeed *= .7;
    // }else if(pivot.getPosition() <=60){
    //   ySpeed *=.7;
    //   xSpeed *= .7;
    // }
    if (driver.getPOV() == 0) {
      drivebase.zeroHeading();
    }
    // if(driver.getPOV() == 180){
    //   autoAllignL.cancel();
    //   autoAllignR.cancel();
    // }
    // if(driver.getYButton()){
    //   autoAllignL.cancel();
    //   autoAllignL = new AutoAllignL();
    //   autoAllignL.initialize();
    //   autoAllignL.schedule();
    // }
    // if(driver.getAButton()){
    //   autoAllignR.cancel();
    //   autoAllignR = new AutoAllignR();
    //   autoAllignR.initialize();
    //   autoAllignR.schedule();
    // }
    // if(driver.getPOV() == 90){
    //   invert = 1;
    // }else if(driver.getPOV() == 270){
    //   invert = -1;
    // }

    if(driver.getBButton()){
      camSystem.poleSide = PoleSide.RIGHT;
      alignToCoral.cancel();
      alignToCoral = new AlignToCoral();
      alignToCoral.initialize();
      alignToCoral.schedule();
      // Double yaw = camSystem.getYawForTag(0, camSystem.lastTag);
      // if(yaw != null){
      //   rot = -yaw * .002 * Constants.DriveConstants.kMaxAngularSpeed;
      // }
      // ArrayList<Double> speeds = camSystem.getPoseToTravel(1);

      // if(!camSystem.side.getSelected()){
      //   xSpeed = -speeds.get(0);
      //   ySpeed = -speeds.get(1);
      // }else{
      //   xSpeed = speeds.get(0);
      //   ySpeed = speeds.get(1);
      // }
      
      // if(Math.abs(speeds.get(0)) < .5 && Math.abs(speeds.get(1)) < .5){
      //   rot = camSystem.getPerpendicularYaw() * .0014 * Constants.DriveConstants.kMaxAngularSpeed;
      // }
      // rot = camSystem.getPerpendicularYaw(0) * .0014 * Constants.DriveConstants.kMaxAngularSpeed;
      // if(Math.abs(rot) < .1){
      //   ArrayList<Double> speeds = camSystem.getPoseToTravel(1);
      //   xSpeed = speeds.get(0) * .6;
      //   ySpeed = speeds.get(1) * .6;
      // }
    }
    if(driver.getXButton()){
      camSystem.poleSide = PoleSide.LEFT;
      alignToCoral.cancel();
      alignToCoral = new AlignToCoral();
      alignToCoral.initialize();
      alignToCoral.schedule();
      // Double yaw = camSystem.getYawForTag(0, camSystem.lastTag);
      // if(yaw != null){
      //   rot = -yaw * .002 * Constants.DriveConstants.kMaxAngularSpeed;
      // }
      // ArrayList<Double> speeds = camSystem.getPoseToTravel(0);
      // xSpeed = speeds.get(0);
      // ySpeed = speeds.get(1);
      // if(!camSystem.side.getSelected()){
      //   xSpeed = -speeds.get(0);
      //   ySpeed = -speeds.get(1);
      // }else{
      //   xSpeed = speeds.get(0);
      //   ySpeed = speeds.get(1);
      // }
      // if(Math.abs(speeds.get(0)) < .5 && Math.abs(speeds.get(1)) < .5){
      //   rot = camSystem.getPerpendicularYaw() * .0014 * Constants.DriveConstants.kMaxAngularSpeed;
      // }
      // rot = camSystem.getPerpendicularYaw(0) * .0014 * Constants.DriveConstants.kMaxAngularSpeed;
      // if(Math.abs(rot) < .1){
      //   ArrayList<Double> speeds = camSystem.getPoseToTravel(0);
      //   xSpeed = speeds.get(0) * .6;
      //   ySpeed = speeds.get(1) * .6;
      // }
    }
    if(driver.getAButton()){
      alignToCoral.cancel();
      alignToCoral = new AlignToCoral();
    }
      drivebase.drive(invert*xSpeed, invert*ySpeed, invert*rot, true);
    
    
    if(operator.getRightTriggerAxis() > 0.5 && mode == "coral" && Timer.getFPGATimestamp() > switchTimer + .5){
      switchTimer = Timer.getFPGATimestamp();
      operator.setRumble(RumbleType.kRightRumble, .5);
      mode = "algae";
      rumbleTimer = Timer.getFPGATimestamp();
      clawZeroPower = .1;
    }else if(operator.getRightTriggerAxis() >0.5 && mode == "algae" && Timer.getFPGATimestamp() > switchTimer + .5){
      switchTimer = Timer.getFPGATimestamp();
      operator.setRumble(RumbleType.kLeftRumble, .5);
      mode = "coral";
      rumbleTimer = Timer.getFPGATimestamp();
      clawZeroPower = .05;
    }else if(Timer.getFPGATimestamp() > rumbleTimer + 0.5){
      operator.setRumble(RumbleType.kRightRumble, 0);
      operator.setRumble(RumbleType.kLeftRumble, 0);
    }

    if(driver.getRightBumperButton()){
      climber.ClimbUp();
      climber.ClimbDeploy();
    }else if(driver.getLeftBumperButton()){
      climber.ClimbDown();
      climber.ClimbRetract();
    }else{
      climber.ClimbZero();
    }
    // if(driver.getBButton()){
    //   Double yaw = camSystem.getYawForTag(0, 18);
    //   if(yaw != null){
    //     rot = -yaw * .002 * Constants.DriveConstants.kMaxAngularSpeed;
    //   }
    // }
    
    

    // if(operator.getPOV() == 0){
    //       pivotCommand.cancel();
    //       pivotCommand = new PivotCommand(PivotState.GROUND);
    //       pivotCommand.initialize();
    //       pivotCommand.schedule();

    //     }else if(operator.getPOV() == 180){
    //       pivotCommand.cancel();
    //       pivotCommand = new PivotCommand(PivotState.HUMANSTATIONINTAKE);
    //       pivotCommand.initialize();
    //       pivotCommand.schedule();

    //     }else if(operator.getPOV() == 270){
    //       pivotCommand.cancel();
    //       pivotCommand = new PivotCommand(PivotState.TRANSITIONSTATE);
    //       pivotCommand.initialize();
    //       pivotCommand.schedule();

    //     }else if(operator.getPOV() == 90){
    //       pivotCommand.cancel();
    //       pivotCommand = new PivotCommand(PivotState.PROCESSOR);
    //       pivotCommand.initialize();
    //       pivotCommand.schedule();

    //      }

    //      if(operator.getYButton()){
    //       elevatorCommand.cancel();
    //       elevatorCommand = new ElevatorCommand(ElevatorState.HIGHALGAEINTAKE);
    //       elevatorCommand.initialize();
    //       elevatorCommand.schedule();
    //      }else if(operator.getBButton()){
    //       elevatorCommand.cancel();
    //       elevatorCommand = new ElevatorCommand(ElevatorState.L4CORALSCORE);
    //       elevatorCommand.initialize();
    //       elevatorCommand.schedule();
    //      }else if(operator.getAButton()){
    //       elevatorCommand.cancel();
    //       elevatorCommand = new ElevatorCommand(ElevatorState.GROUND);
    //       elevatorCommand.initialize();
    //       elevatorCommand.schedule();
    //      }else if(operator.getXButton()){
    //       elevatorCommand.cancel();
    //       elevatorCommand = new ElevatorCommand(ElevatorState.HUMANSTATIONINTAKE);
    //       elevatorCommand.initialize();
    //       elevatorCommand.schedule();
    //      }


    if(mode == "coral"){
      if(operator.getPOV() == 0){
        CancelCommands();
        L4CoralScore.initialize();
        L4CoralScore.schedule();
      }else if(operator.getPOV() == 180){
        CancelCommands();
        L1CoralScore.initialize();
        L1CoralScore.schedule();
      }else if(operator.getPOV() == 270){
        CancelCommands();
        L2CoralScore.initialize();
        L2CoralScore.schedule();
      }else if(operator.getPOV() == 90){
        CancelCommands();
        L3CoralScore.initialize();
        L3CoralScore.schedule();
      }
    }else if(mode == "algae"){
      if(operator.getPOV() == 0){
        CancelCommands();
        highAlgaeIntake.initialize();
        highAlgaeIntake.schedule();
      }else if(operator.getPOV() == 180){
        CancelCommands();
        processorScore.initialize();
        processorScore.schedule();
      }else if(operator.getPOV() == 270){
        CancelCommands();
        lowAlgaeIntake.initialize();
        lowAlgaeIntake.schedule();
      }else if(operator.getPOV() == 90){
        CancelCommands();
        pivot.setPivotState(PivotState.SIGMATEST);
        elevator.setElevatorState(ElevatorState.L4CORALSCORE);
        wristCommand = new WristCommand(WristState.NET);
        wristCommand.initialize();
        wristCommand.schedule();
      }
    }
    if(operator.getYButton()){
      CancelCommands();
      humanPlayerIntake.initialize();
      humanPlayerIntake.schedule();
    }else if(operator.getAButton()){
      CancelCommands();
      groundAlgaeIntake.initialize();
      groundAlgaeIntake.schedule();
    }else if(operator.getXButton()){
      CancelCommands();
      pivotCommand = new PivotCommand(PivotState.CLIMB);
      pivotCommand.initialize();
      pivotCommand.schedule();
    }

    if(operator.getRightBumperButton()){
      claw.clawOn(1);
    }else if(operator.getLeftBumperButton()){
      claw.clawReverse(1);
    }else{
      claw.clawReverse(clawZeroPower);
    }
    
    if(operator.getLeftTriggerAxis() > .5){
      CancelCommands();
      transition.initialize();
      transition.schedule();
    }
    if(operator.getXButton()){
      pivotCommand = new PivotCommand(PivotState.CLIMB);
      pivotCommand.initialize();
      pivotCommand.schedule();
    }
    if(operator.getBButton()){
     pivot.setPivotState(PivotState.SHOOTINGNET);
    }
    // if(operator.getBButton()){
    //   elevator.setElevatorState(ElevatorState.L4CORALSCORE);
    //   pivot.setPivotState(PivotState.SHOOTINGNET);
    // }
    
  }
  public void CancelCommands(){
   // transTimer = Timer.getFPGATimestamp();
  // groundCoralIntake.cancel();
    groundAlgaeIntake.cancel();
    humanPlayerIntake.cancel();
    highAlgaeIntake.cancel();
    lowAlgaeIntake.cancel();
    processorScore.cancel();
    L3CoralScore.cancel();
    L4CoralScore.cancel();
    L2CoralScore.cancel();
    L1CoralScore.cancel();
    transition.cancel();
  }
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}