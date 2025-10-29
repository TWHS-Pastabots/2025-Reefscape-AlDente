package frc.robot;


import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DomainExpansion.HumanAllign;
import frc.robot.Commands.DomainExpansion.AutoAllignL;
import frc.robot.Commands.DomainExpansion.AutoAllignR;
import frc.robot.Commands.DomainExpansion.GroundAlgaeIntake;
import frc.robot.Commands.DomainExpansion.GroundIntakeCoral;
import frc.robot.Commands.DomainExpansion.GroundSequence;
import frc.robot.Commands.DomainExpansion.HighAlgaeIntake;
import frc.robot.Commands.DomainExpansion.HumanPlayerIntake;
import frc.robot.Commands.DomainExpansion.Intake;
import frc.robot.Commands.DomainExpansion.L1CoralScore;
// import frc.robot.Commands.DomainExpansion.L2CoralScore1;
// import frc.robot.Commands.DomainExpansion.L3CoralScore1;
import frc.robot.Commands.DomainExpansion.L4CoralScore;
// import frc.robot.Commands.DomainExpansion.L4CoralScore1;
import frc.robot.Commands.DomainExpansion.L3coraly;
import frc.robot.Commands.DomainExpansion.L3CoralScore;
import frc.robot.Commands.DomainExpansion.L2CoralScore;
import frc.robot.Commands.DomainExpansion.L2coraly;
// import frc.robot.Commands.DomainExpansion.L4coraly;
import frc.robot.Commands.DomainExpansion.LowAlgaeIntake;
import frc.robot.Commands.DomainExpansion.MovetoTransistion;
import frc.robot.Commands.DomainExpansion.NetScore;
import frc.robot.Commands.DomainExpansion.NewGround;
import frc.robot.Commands.DomainExpansion.Outtake;
import frc.robot.Commands.DomainExpansion.ProcessorScore;
import frc.robot.Commands.DomainExpansion.Tra1;
import frc.robot.Commands.DomainExpansion.Transitio;
import frc.robot.Commands.DomainExpansion.Transition;
import frc.robot.Commands.DomainExpansion.Outtakedos;
import frc.robot.Commands.AlignToCoral;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.subsystems.claw.Wrist;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.VectorPlate;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.CameraSystem;
import frc.robot.subsystems.vision.CameraSystem.PoleSide;
import frc.robot.subsystems.IO.*;;

public class Robot extends LoggedRobot {
  // all the initialing for the systems
  public double cameraSame;
  public double speedMod;
  public double clawZeroPower;
  private Climber climber;
  private DriveSubsystem drivebase;
  private Wrist wrist;
  private Claw claw;
  private Pivot pivot; 
  private Elevator elevator;
  private CameraSystem camSystem;
  private VectorPlate vectorPlate;
  private DigitalInputs digitalInput;
  private GroundAlgaeIntake groundAlgaeIntake;
  private GroundIntakeCoral groundCoralIntake;
  private HighAlgaeIntake highAlgaeIntake;
  private HumanPlayerIntake humanPlayerIntake;
  private NewGround newground;
  private L1CoralScore L1CoralScore;
  private L2CoralScore L2CoralScore;
  private L3CoralScore L3CoralScore;
  private L4CoralScore L4CoralScore;
  private Outtakedos outtakedos;
  private MovetoTransistion movetoTransistion;
  private LowAlgaeIntake lowAlgaeIntake;
  private ProcessorScore processorScore;
  private PivotCommand pivotCommand;
  private ElevatorCommand elevatorCommand;
  private Transition l4Transition;
  private Transitio transition;
  private Outtake outtake;
  private Intake intake;
  private HumanAllign humanAllign;
  private AutoAllignR autoAllignR;
  private AutoAllignL autoAllignL;
  private NetScore netScore;

  private GroundSequence groundSequence;
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
  double timie = 0.5;
  
  public PIDController xController;
  public PIDController yController;
  public PIDController thetaController;

  private boolean usingAlign;
  // that is a chooser for the autons utilizing the sendableChooser which allows
  // us to choose the auton commands
  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  @Override
  public void robotInit() {
    Elevator.adjuster =0;
    clawZeroPower = 0;
    speedMod = 1;
    vectorPlate = VectorPlate.getInstance();
    drivebase = DriveSubsystem.getInstance();
    elevator = Elevator.getInstance();
    wrist = Wrist.getInstance();
    climber = Climber.getInstance();
    claw = Claw.getInstance();
    pivot = Pivot.getInstance();
    camSystem = CameraSystem.getInstance(); 
    digitalInput = DigitalInputs.getInstance();
    camSystem.AddCamera(new PhotonCamera("ClimbCam"), new Transform3d(
      new Translation3d(-0.30043, -0.26457, 0.31945), new Rotation3d(0.0, 0.0, Math.toRadians(-55.56095))), 
      true);
    camSystem.AddCamera(new PhotonCamera("SwerveCam"), new Transform3d(
      new Translation3d(0.28831, -0.2421, 0.29561), new Rotation3d(0.0, Math.toRadians(2.5), Math.toRadians(-105))), 
      true);
    
    pivotCommand = new PivotCommand(PivotState.TRANSITIONSTATE);
    elevatorCommand = new ElevatorCommand(ElevatorState.TEST);
    groundAlgaeIntake = new GroundAlgaeIntake();
    groundCoralIntake = new GroundIntakeCoral();
    highAlgaeIntake = new HighAlgaeIntake();
    humanPlayerIntake = new HumanPlayerIntake();
    L1CoralScore = new L1CoralScore();
    L2CoralScore = new L2CoralScore();
    L3CoralScore = new L3CoralScore();
    L4CoralScore = new L4CoralScore();
    lowAlgaeIntake = new LowAlgaeIntake();
    processorScore = new ProcessorScore();
    transition = new Transitio();
    l4Transition = new Transition();
    intake = new Intake();
    outtake = new Outtake();
    outtakedos = new Outtakedos();
    autoAllignR = new AutoAllignR();
    autoAllignL = new AutoAllignL();
    humanAllign = new HumanAllign();
    netScore = new NetScore();






    movetoTransistion = new MovetoTransistion();

    elevator.elevatorMotorR.clearFaults();
    
    newground = new NewGround();

    groundSequence = new GroundSequence();
    //alignToCoral = new AlignToCoral();
    // x: .0095 y: .95 theta: .008

    xController = new PIDController(.0095, 0, 0);
    yController = new PIDController(2.55 , 0, 0);
    thetaController = new PIDController(0.015, 0, 0);

    thetaController.enableContinuousInput(0, 360);

    xController.setTolerance(.005);
    yController.setTolerance(.005);
    thetaController.setTolerance(.01);

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
    NamedCommands.registerCommand("L4 transition", l4Transition);
    NamedCommands.registerCommand("L2Score", L2CoralScore);
    NamedCommands.registerCommand("HumanIntake", humanPlayerIntake);
    NamedCommands.registerCommand("L3Score", L3CoralScore);
    NamedCommands.registerCommand("HighAlgae", highAlgaeIntake);
    NamedCommands.registerCommand("LowAlgae", lowAlgaeIntake);
    NamedCommands.registerCommand("ProcessorScore", processorScore);
    NamedCommands.registerCommand("Elevator", elevatorCommand);
    NamedCommands.registerCommand("Transition", transition);
    NamedCommands.registerCommand("outtake", outtake);
    NamedCommands.registerCommand("outtake2", outtakedos);
    NamedCommands.registerCommand("groundSequence", groundSequence);
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("AutoAllignR", autoAllignR);
    NamedCommands.registerCommand("AutoAllignL", autoAllignL);
    NamedCommands.registerCommand("NetScore", netScore);
    NamedCommands.registerCommand("TransitionToHuman", movetoTransistion);
    NamedCommands.registerCommand("CoralAllignL", new AlignToCoral(PoleSide.LEFT));
    NamedCommands.registerCommand("CoralAlignR", new AlignToCoral(PoleSide.RIGHT));
    NamedCommands.registerCommand("HumanAllign", new HumanAllign());
    m_chooser.addOption("middlePath", new PathPlannerAuto("middlePath"));
    m_chooser.addOption("1.eR.fR.fL", new PathPlannerAuto("1.eR.fR.fL"));
    m_chooser.addOption("testReal", new PathPlannerAuto("testReal"));
    m_chooser.addOption("test", new PathPlannerAuto("test"));
    m_chooser.addOption("testcoral", new PathPlannerAuto("testcoral"));
    m_chooser.addOption("3_C_2_P2C align", new PathPlannerAuto("3_C_2_P2C align"));
    m_chooser.addOption("no.5path", new PathPlannerAuto("no.5path"));
    m_chooser.addOption("1G.eR.fR.aL", new PathPlannerAuto("1G.eR.fR.aL"));
    SmartDashboard.putData(m_chooser);
    
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("adjuster", Elevator.adjuster);
    SmartDashboard.putNumber("MotorL Current", wrist.MotorL.getOutputCurrent());
    SmartDashboard.putNumber("feedforwards/feedforwardL", wrist.feedforwardL.calculate(2*Math.PI*wrist.MotorL.getAbsoluteEncoder().getPosition(), 0));
    SmartDashboard.putNumber("feedforwards/feedforwardR", wrist.feedforwardR.calculate(2*Math.PI*wrist.MotorL.getAbsoluteEncoder().getPosition(), 0));
    SmartDashboard.putNumber("rot abs encoder", wrist.MotorL.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("tilt abs encoder", wrist.MotorR.getAbsoluteEncoder().getPosition());
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
    SmartDashboard.putNumber("lasttag",camSystem.lastTag);
    SmartDashboard.putBoolean("zryu", camSystem.hasDesiredTarget(1, camSystem.lastTag));
    SmartDashboard.putBoolean("ended", new AlignToCoral(PoleSide.LEFT).isFinished());


    SmartDashboard.putNumber("pivot feedforward", pivot.feedForward.calculate(Math.toRadians(pivot.pivotMotor.getAbsoluteEncoder().getPosition()),0));

    SmartDashboard.putNumber("elevator feedforward", elevator.feedForward.calculate(elevator.elevatorMotorL.getEncoder().getPosition(), 0));
    SmartDashboard.putNumber("elevator encoder", elevator.getElevatorPosition());
    SmartDashboard.putBoolean("elevator Top Switch", elevator.getTopLimitSwitch());
    SmartDashboard.putBoolean("elevator Bot Switch", elevator.getBottomLimitSwitch());
    // SmartDashboard.putBoolean("algae Breakbeam", claw.getAlgaeBreakBeam());
    // SmartDashboard.putBoolean("coral Breakbeam", claw.getCoralBreakBeam());
    SmartDashboard.putString("wristlaststate", wrist.lastState.toString());
    SmartDashboard.putBoolean("pivotisfinished", pivotCommand.isFinished());
    SmartDashboard.putNumber("Last Tag Seen", camSystem.lastTag);
    SmartDashboard.putNumber("Desired Degree", 
    CameraSystem.aprilTagFieldLayout.getTagPose(18).get().getRotation().toRotation2d().getDegrees());
    SmartDashboard.putNumber("Currenr Degree", DriveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees());

    //Test for position


    //test
    SmartDashboard.putBoolean("Transition ready ", groundCoralIntake.transitionReady);
    //SmartDashboard.putNumber("Ground Timer ", groundCoralIntake.timer);


    if(camSystem.getTargetRange(1, camSystem.lastTag) != null && camSystem.getYawForTag(1, camSystem.lastTag)!= null)
    {
      SmartDashboard.putNumber("SwerveCam dist", camSystem.getTargetRange(1, camSystem.lastTag));
      SmartDashboard.putNumber("SwerveCam yaw", camSystem.getYawForTag(1, camSystem.lastTag));
      SmartDashboard.putNumber("SwerveCam Pitch", camSystem.getPitch(1, camSystem.lastTag));
    }
    if(camSystem.getTargetRange(0, camSystem.lastTag) != null && camSystem.getYawForTag(0, camSystem.lastTag)!= null)
    {
      SmartDashboard.putNumber("ClimbCam dist", camSystem.getTargetRange(0, camSystem.lastTag));
      SmartDashboard.putNumber("ClimbCam yaw", camSystem.getYawForTag(0, camSystem.lastTag));
      SmartDashboard.putNumber("CLimbCam Pitch", camSystem.getPitch(0, camSystem.lastTag));
    }
    SmartDashboard.putNumber("heading", drivebase.getWorkingHeading());
    if(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null
    && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null)
    {
      SmartDashboard.putNumber("xController", xController.calculate(
        camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()));
      SmartDashboard.putNumber("yController", yController.calculate(
        camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()));
      SmartDashboard.putNumber("thetaController", thetaController.calculate(drivebase.getWorkingHeading()));
    }
    
    SmartDashboard.putString("Focus Cam", (camSystem.focusCamIndex == 0)  
    ? "climb" : "swerve");
    // SmartDashboard.putNumber("Currenr Degree", CameraSystem.get);


    SmartDashboard.putBoolean("Beam2", digitalInput.getInputs()[2]);
    SmartDashboard.putBoolean("Beam3", digitalInput.getInputs()[3]);

    // SmartDashboard.putNumber("target tilt", Wrist.wristState.tilt);

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

    CancelCommands();
    transition.initialize();
    transition.schedule();
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
    elevator.elevatorMotorR.clearFaults();
    boolean atRight = false;
    double multFactor = 1;
    usingAlign = false;
    elevator.updatePose();
    pivot.updatePose();
    

    
    double ySpeed = drivebase.inputDeadband(-driver.getLeftX()) * speedMod;
    double xSpeed = drivebase.inputDeadband(-driver.getLeftY()) * speedMod;
    double rot = drivebase.inputDeadband(-driver.getRightX()) * speedMod;
    if(driver.getRightTriggerAxis() >= .5){
      speedMod = .35;
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

    if(driver.getBButton()){
      usingAlign = true;
      camSystem.poleSide = PoleSide.RIGHT;
      thetaController.setSetpoint(0);
      //was .11
      yController.setSetpoint(0.04);
      xController.setSetpoint(-1.1);
      camSystem.focusCamIndex = 1;
      if (camSystem.hasDesiredTarget(0, camSystem.lastTag) && camSystem.hasDesiredTarget(1, camSystem.lastTag)) 
      {
        updateThetaControllerSetpoint(camSystem.lastTag);
        
        xSpeed = xController.calculate(camSystem.getYawForTag(1, camSystem.lastTag));
        ySpeed = yController.calculate(camSystem.getTargetRange(1, camSystem.lastTag).doubleValue());
        multFactor = 2.4;
        multFactor = 1;
        rot = thetaController.calculate(drivebase.getWorkingHeading());
      }
    }

    
    if(driver.getXButton()){
      usingAlign = true;
      camSystem.poleSide = PoleSide.LEFT;
      //thetaController.setSetpoint(0);
      //was .11
      
      yController.setSetpoint(0.235);//was.186
      xController.setSetpoint(-15.8); //was -18.8
      updateXControllerSetpoint();
      camSystem.focusCamIndex = 0;

     if(camSystem.focusCamIndex == 0 && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null){
        updateThetaControllerSetpoint(camSystem.lastTag);
        xSpeed =  xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag));
        ySpeed =  yController.calculate(camSystem.getTargetRange(0, camSystem.lastTag).doubleValue());
        rot = thetaController.calculate(drivebase.getWorkingHeading());
      }
    }
    
    // if(driver.getXButton()){
    //   usingAlign = true;
    //   camSystem.poleSide = PoleSide.LEFT;
    //   //thetaController.setSetpoint(0);
    //   //was .11
      
    //   yController.setSetpoint(0.215);//was.186
    //   xController.setSetpoint(-15.8); //was -18.8
    //   // updateXControllerSetpoint();
    //   camSystem.focusCamIndex = 0;

    //  if(camSystem.focusCamIndex == 0 && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null){
    //     updateThetaControllerSetpoint(camSystem.lastTag);
    //     xSpeed =  xController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag));
    //     ySpeed =  yController.calculate(camSystem.getTargetRange(0, camSystem.lastTag).doubleValue());
    //     rot = thetaController.calculate(drivebase.getWorkingHeading());
    //   }
    // }
    drivebase.drive(xSpeed, multFactor * ySpeed, rot, !usingAlign);
    
    
    // if(operator.getLeftStickButtonPressed()){
    //   Elevator.adjuster = Elevator.adjuster -.2;
    //   elevator.UpdateEnumPoses();
    // }
    if(operator.getRightStickButtonPressed() && elevator.getState() == ElevatorState.GROUND){
      Elevator.adjuster = Elevator.adjuster +.2;
      elevator.UpdateEnumPoses();
      // elevator.elevatorMotorL.getEncoder().setPosition(0);
      // elevator.elevatorMotorR.getEncoder().setPosition(0);
    }

    if(driver.getAButton())
    {
     // vectorPlate.setVectorState(VectorState.DOWN);
      vectorPlate.ClimbOn();

    }
    else{
      vectorPlate.turnOff();
    }

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
      clawZeroPower = .0;
    }else if(Timer.getFPGATimestamp() > rumbleTimer + 0.5){
      operator.setRumble(RumbleType.kRightRumble, 0);
      operator.setRumble(RumbleType.kLeftRumble, 0);
    }

    if(driver.getRightBumper()){
      climber.ClimbDown();
    }else if (driver.getLeftBumperButton()){
      climber.ClimbUp();
    }else{
      climber.ClimbZero();
    }
    
      
      if(driver.getYButton())
      {
        CancelCommands();
        humanAllign.initialize();
        humanAllign.schedule();
    }


    


    if(mode.equals( "coral")){
      if(operator.getPOV() == 0){
        CancelCommands();
        L4CoralScore.initialize();
        L4CoralScore.schedule();

      }
        else if(operator.getPOV() == 180){
         CancelCommands();
         L1CoralScore.initialize();
         L1CoralScore.schedule();
          wrist.lastState = WristState.L1CORALSCORE;
        }else if(operator.getPOV() == 270){
        CancelCommands();
        L2CoralScore.initialize();
        L2CoralScore.schedule();
      }else if(operator.getPOV() == 90){
        CancelCommands();
        L3CoralScore.initialize();
        L3CoralScore.schedule();
      }
      if(operator.getAButton()){
        CancelCommands();    
          groundSequence.initialize();
          groundSequence.schedule();
        }

    if(operator.getRightBumperButton()){
      claw.flywheelReverse(1);
       claw.clawOn(-.5);
    }else if(operator.getLeftBumperButton()&&digitalInput.getInputs()[2]){//intake
      claw.flywheelReverse(0.8);
      claw.clawReverse(.5);
    }else{
      claw.flywheelOff();
      claw.clawOff();
    }
    }
    else if(mode.equals( "algae")){
      if(operator.getPOV() == 0){
        CancelCommands();
        netScore.initialize();
        netScore.schedule();
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
        highAlgaeIntake.initialize();
        highAlgaeIntake.schedule();
      }

    if(operator.getLeftBumperButton()){
      claw.flywheelOn(0.8);
      claw.clawOn(0.5);
    }
    else if(operator.getRightBumperButton()){
      claw.flywheelOn(-0.8);
      claw.clawOn(-.5);
    }
    else{
      claw.flywheelOff();
      claw.clawOn(.15);
    }
  }
  if(operator.getYButton()){
    CancelCommands();
    humanPlayerIntake.initialize();
    humanPlayerIntake.schedule();
  }


   if(operator.getAButton()){
    CancelCommands();    
      groundSequence.initialize();
      groundSequence.schedule();
    }

  if(operator.getXButton()){
    CancelCommands();
    pivotCommand = new PivotCommand(PivotState.CLIMB);
    pivotCommand.initialize();
    pivotCommand.schedule();
  }
    
    if(operator.getLeftStickButton()){
      CancelCommands();
      l4Transition.initialize();
      l4Transition.schedule();
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
    // if(operator.getBButton()){
    //  CancelCommands();
    //  l4Transition.initialize();
    //  l4Transition.schedule();
    // }
  
    // if(operator.getBButton()){
    //   elevator.setElevatorState(ElevatorState.L4CORALSCORE);
    //   pivot.setPivotState(PivotState.SHOOTINGNET);
    // }
    
  }
  public void CancelCommands(){
   // transTimer = Timer.getFPGATimestamp();
    groundAlgaeIntake.cancel();
    newground.cancel();
    groundCoralIntake.cancel();
    humanPlayerIntake.cancel();
    highAlgaeIntake.cancel();
    lowAlgaeIntake.cancel();
    processorScore.cancel();
    autoAllignL.cancel();
    autoAllignR.cancel();
    L3CoralScore.cancel();
    L4CoralScore.cancel();
    L2CoralScore.cancel();
    L1CoralScore.cancel();
    l4Transition.cancel();
    transition.cancel();
    netScore.cancel();
    groundSequence.cancel();
    outtake.cancel();
    outtakedos.cancel();
    intake.cancel();
    humanAllign.cancel();
    movetoTransistion.cancel();
    autoAllignL.cancel();
    autoAllignR.cancel();
    processorScore.cancel();
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
  private void updateThetaControllerSetpoint(int targetID) {
    switch (targetID) {
      case 6, 19 -> thetaController.setSetpoint(30);
      case 7, 18 -> thetaController.setSetpoint(90);
      case 8, 17 -> thetaController.setSetpoint(150);
      case 9, 22 -> thetaController.setSetpoint(210);
      case 10, 21 -> thetaController.setSetpoint(270);
      case 11, 20 -> thetaController.setSetpoint(330);
    }

    PPHolonomicDriveController.clearXFeedbackOverride();
    PPHolonomicDriveController.clearYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }
  private void updateXControllerSetpoint(){
    if(camSystem.focusCamIndex == 0 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null){
      double range = camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag);
      xController.setSetpoint(-579.92142 * (Math.pow(range, 2)) + 532.84093 * (range) - 99.40781);//reg
      // xController.setSetpoint(-184.449 * (Math.pow(range, 2)) + 279.54 * (range) - 78.71919);//reg

    }
    else if(camSystem.focusCamIndex == 1 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null) {
      xController.setSetpoint(-20.8);
    }
    else if(camSystem.focusCamIndex == 2 && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null) {
      xController.setSetpoint(-67);
    }
  }
}