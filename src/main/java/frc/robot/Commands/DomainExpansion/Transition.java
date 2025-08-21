// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Wrist;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Transition extends Command {
  /** Creates a new Transition. */
  public double timer;
  private Wrist wristData;
  private WristCommand wrist;
  private PivotCommand pivot;
  private ElevatorCommand elevator;
  private boolean ended;
  private Pivot pivotData;
  public double delay;
  public Transition() {
    // Use addRequirements() here to declare subsystem dependencies.
    wristData = Wrist.getInstance();
    wrist = new WristCommand(WristState.TRANSITIONSTATE);
    pivot = new PivotCommand(PivotState.TRANSITIONSTATE);
    elevator = new ElevatorCommand(ElevatorState.GROUND);
    timer = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
    timer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pivot.schedule();
    if(wristData.lastState == WristState.LOWALGAEINTAKE){
      delay = 4;
    }else if(wristData.lastState == WristState.HIGHALGAEINTAKE){
      delay = 1.75;
    }
    else{
      delay =0;
    }
    if(wristData.lastState == WristState.L4CORALSCORE || wristData.lastState == WristState.L3CORALSCORE 
    || wristData.lastState == WristState.L2CORALSCORE || wristData.lastState == WristState.HUMANSTATIONINTAKE){
      pivot.initialize();
      pivot.schedule();
      elevator.initialize();
      elevator.schedule();
      wrist.initialize();
      wrist.schedule();
      if(wrist.isFinished() && pivot.isFinished() && elevator.isFinished())
    {
      ended = true;
    }
    }else if(wristData.lastState == WristState.GROUND || wristData.lastState == WristState.LOWALGAEINTAKE 
    || wristData.lastState == WristState.HIGHALGAEINTAKE){
      pivot.initialize();
      pivot.schedule();
      
    }
    else{
      wrist.initialize();
      wrist.schedule();
    if((Math.abs(wristData.getVoltageLeft()) < .5 && Math.abs(wristData.getVoltageRight()) < .5 )){
      pivot.initialize();
      pivot.schedule();
    }
    
    if(pivot.isFinished() && (Math.abs(wristData.getVoltageLeft()) < .5 && Math.abs(wristData.getVoltageRight()) < .5)){
      elevator.initialize();
      elevator.schedule();
    }
    if(wrist.isFinished() && pivot.isFinished() && elevator.isFinished())
    {
      ended = true;
    }
    }
    //else if(wristData.lastState == WristState.GROUND){
    //   pivot.initialize();
    //   pivot.schedule();
    //   elevator.initialize();
    //   elevator.schedule();
    //   if(Timer.getFPGATimestamp() > timer + delay){
    //     wrist.initialize();
    //     wrist.schedule();
    //     if(wrist.isFinished()){
    //       ended = true;
    //     }
    //   }
      
    // }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setLastState();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
