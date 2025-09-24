// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DomainExpansion;

import frc.robot.Commands.DomainExpansion.GroundIntakeCoral;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundSequence extends SequentialCommandGroup {
  /** Creates a new GroundSequence. */
  private GroundIntakeCoral groundIntakeCoral;
  private WaitCommand waitCommand;
  private NewGround newGround;
  public GroundSequence() {
    // Add your commands in the addCommands() call, e.g.
    groundIntakeCoral = new GroundIntakeCoral();
    newGround = new NewGround();
    waitCommand = new WaitCommand(0.2);
    addCommands(groundIntakeCoral, waitCommand, newGround);

  }
}
