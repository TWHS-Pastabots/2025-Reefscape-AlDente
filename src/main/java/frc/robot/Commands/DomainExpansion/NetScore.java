// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NetScore extends Command {
  /** Creates a new NetScore. */
  private WristCommand wrist;
  private PivotCommand pivot;
  private ElevatorCommand elevatorCommand;
  private NetScore netScore;
  private boolean ended;
  private boolean transitionReady;
  private Claw claw;
  public NetScore() {
    wrist = new WristCommand(WristState.NET);
    pivot = new PivotCommand(PivotState.SHOOTINGNET);
    elevatorCommand = new ElevatorCommand(ElevatorState.L4CORALSCORE);
    claw = Claw.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.initialize();
    pivot.initialize();
    elevatorCommand.initialize();
    claw = Claw.getInstance();
    transitionReady = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!transitionReady)
    {
      pivot.schedule();
      
      // if(pivot.isFinished()){
        wrist.schedule();
        elevatorCommand.schedule();
      
    }
      // if(pivot.isFinished() && wrist.isFinished()){
      //   claw.clawReverse(.6);
      // }
      if(pivot.isFinished() && elevatorCommand.isFinished() && wrist.isFinished())
    {
      ended = true;
    }
  }
  //   pivot.schedule();
  //   wrist.schedule();
  //   elevatorCommand.schedule();
  //   if(pivot.isFinished() && elevatorCommand.isFinished() && wrist.isFinished())
  //   {
  //     ended = true;
  //   }
  // }

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
