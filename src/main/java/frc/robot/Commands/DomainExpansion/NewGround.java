// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NewGround extends Command {
  private WristCommand wrist;
  private PivotCommand pivot;
  private ElevatorCommand elevator;
  private boolean ended;
   private WaitCommand waitCommand;
  public boolean transitionReady;
  /** Creates a new NewGround. */
  public NewGround() {
    wrist = new WristCommand(WristState.GROUND);
    pivot = new PivotCommand(PivotState.GROUND);
    elevator = new ElevatorCommand(ElevatorState.GROUND);
    waitCommand = new WaitCommand(2);
    
    transitionReady = true;


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.initialize();
    pivot.initialize();
    elevator.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    waitCommand.initialize();
    elevator.schedule();
    pivot.schedule();
    wrist.schedule();
    waitCommand.schedule();
    // while(!waitCommand.isFinished()){}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
