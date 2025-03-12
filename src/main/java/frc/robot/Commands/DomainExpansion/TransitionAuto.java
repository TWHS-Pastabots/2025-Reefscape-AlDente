// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransitionAuto extends Command {
  private WristCommand wrist;
  private PivotCommand pivot;
  private ElevatorCommand elevator;
  private boolean ended;
  /** Creates a new TransitionAuto. */
  public TransitionAuto() {
  

  wrist = new WristCommand(WristState.TRANSITIONSTATE);
  pivot = new PivotCommand(PivotState.TRANSITIONSTATE);
  elevator = new ElevatorCommand(ElevatorState.AUTONTRANSITION);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.initialize();
    wrist.initialize();
    elevator.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.schedule();
    wrist.schedule();
    if(wrist.isFinished()){
      elevator.schedule();
      if(elevator.isFinished()){
        ended = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
