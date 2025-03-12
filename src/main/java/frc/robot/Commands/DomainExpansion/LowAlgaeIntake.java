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
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowAlgaeIntake extends Command {
  private WristCommand wrist;
  private PivotCommand pivot;
  private ElevatorCommand elevator;
  private boolean ended;
  private boolean transitionReady;
  private Claw claw;
  /** Creates a new GroundIntakeCoral. */
  public LowAlgaeIntake() {
    wrist = new WristCommand(WristState.LOWALGAEINTAKE);
    pivot = new PivotCommand(PivotState.LOWALGAEINTAKE);
    elevator = new ElevatorCommand(ElevatorState.LOWALGAEINTAKE);
    claw = Claw.getInstance();
    transitionReady = false;
    // Use addRequirements() here to declare subsystem dependencies.
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
    if(!transitionReady)
    {
      pivot.schedule();
      elevator.schedule();
      if(pivot.isFinished())
      {
        // claw.clawOn(.4);
        wrist.schedule();
      }
    }
    // if(pivot.isFinished() && elevator.isFinished() && wrist.isFinished() && !transitionReady)
    // {
    //   pivot = new PivotCommand(PivotState.TRANSITIONSTATE);
    //   wrist = new WristCommand(WristState.TRANSITIONSTATE);
    //   pivot.initialize();
    //   wrist.initialize();
    //   transitionReady = true;
    //   wrist.schedule();
    // }
    // if(transitionReady && wrist.isFinished())
    // {
    //   pivot.schedule();
    // }
    // if(pivot.isFinished() && elevator.isFinished() && wrist.isFinished() && transitionReady)
    // {
    //   ended = true;
    // }
    if(pivot.isFinished() && elevator.isFinished() && wrist.isFinished())
    {
      ended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //claw.clawOff();
    wrist.setLastState();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
