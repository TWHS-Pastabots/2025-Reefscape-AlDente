package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;
public class L4CoralScore extends SequentialCommandGroup{
    public L4CoralScore(){
        addCommands(new ElevatorCommand(ElevatorState.L4CORALSCORE),new WaitCommand(0.2), new WristCommand(WristState.L4CORALSCORE), new PivotCommand(PivotState.L4CORALSCORE));
    }
}
