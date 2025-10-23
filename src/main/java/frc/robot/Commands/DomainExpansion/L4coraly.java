package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.PivotCommand;
import frc.robot.Commands.WristCommand;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.pivot.Pivot.PivotState;

public class L4coraly extends ParallelDeadlineGroup{
    public L4coraly(){
        super(new WaitCommand(.5) );

        addCommands(new WristCommand(WristState.L4CORALSCORE), new ElevatorCommand(ElevatorState.L4CORALSCORE) );
    }
}