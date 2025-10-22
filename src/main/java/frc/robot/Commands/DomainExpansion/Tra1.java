// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Tra1 extends ParallelDeadlineGroup {
  /** Creates a new Tra1. */
  public Tra1() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(.3));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PivotCommand(PivotState.TRANSITIONSTATE), new ElevatorCommand(ElevatorState.GROUND));
  }
}
