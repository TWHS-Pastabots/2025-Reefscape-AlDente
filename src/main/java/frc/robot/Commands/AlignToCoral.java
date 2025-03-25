// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.CameraSystem;
import frc.robot.subsystems.vision.CameraSystem.PoleSide;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCoral extends Command {
  /** Creates a new AlignToCoral. */
  private CameraSystem camSystem;
  private DriveSubsystem driveBase;

  private PoleSide side;
  private boolean ended;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  public AlignToCoral() {
    camSystem = CameraSystem.getInstance();
    driveBase = DriveSubsystem.getInstance();
    side = camSystem.poleSide;
    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    thetaController = new PIDController(0, 0, 0);
    
    xController.setSetpoint(0.11);
    thetaController.setSetpoint(0);
    if(side == PoleSide.LEFT){
      yController.setSetpoint(0);
      camSystem.focusCamIndex = 0;
    }
    else {
      yController.setSetpoint(0);
      camSystem.focusCamIndex = 1;
    }
    
    thetaController.enableContinuousInput(-180, 180);

    xController.setTolerance(.01);
    yController.setTolerance(.01);
    thetaController.setTolerance(.1);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (camSystem.hasDesiredTarget(0, camSystem.lastTag) && camSystem.hasDesiredTarget(1, camSystem.lastTag)
    && camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag) != null 
    && camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag) != null) // driver align right so left camera
    {
      updateThetaControllerSetpoint(camSystem.lastTag);

      driveBase.drive(
        -xController.calculate(camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue()),
        camSystem.getTargetRange(camSystem.focusCamIndex, camSystem.lastTag).doubleValue() < 0.4
          ? yController.calculate(camSystem.getYawForTag(camSystem.focusCamIndex, camSystem.lastTag))
          : 0,
        thetaController.calculate(driveBase.getHeading()),
        false);
    } 
    else if(!camSystem.hasDesiredTarget(camSystem.focusCamIndex, camSystem.lastTag)){
      Double yaw = camSystem.getYawForTag(2, camSystem.lastTag);
      // Double yaw = null;
      // if(camSystem.focusCamIndex == 0){
      //   yaw = camSystem.getYawForTag(1, camSystem.lastTag);
      // }
      // else{
      //   yaw = camSystem.getYawForTag(0, camSystem.lastTag);
      // }
      if(yaw != null){
        driveBase.drive(0, 0, -yaw,false);
      }
    }
    else{
      driveBase.drive(0, 0, 0, true);
    }
    
    if(xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()){
      ended = true;
    }
  }
  private void updateThetaControllerSetpoint(int targetID) {
    switch (targetID) {
      case 6, 19 -> thetaController.setSetpoint(300);
      case 7, 18 -> thetaController.setSetpoint(0);
      case 8, 17 -> thetaController.setSetpoint(60);
      case 9, 22 -> thetaController.setSetpoint(120);
      case 10, 21 -> thetaController.setSetpoint(180);
      case 11, 20 -> thetaController.setSetpoint(240);
    }

    PPHolonomicDriveController.clearXFeedbackOverride();
    PPHolonomicDriveController.clearYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0, 0, 0, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}