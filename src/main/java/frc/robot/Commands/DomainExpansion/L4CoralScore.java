package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PivotCommand;
import frc.robot.subsystems.pivot.Pivot.PivotState;
public class L4CoralScore extends SequentialCommandGroup{
    public L4CoralScore(){
        addCommands(new L4coraly(), new PivotCommand(PivotState.L4CORALSCORE));
    }
}
