package frc.robot.Commands.DomainExpansion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PivotCommand;
import frc.robot.subsystems.pivot.Pivot.PivotState;
public class L2CoralScore extends SequentialCommandGroup{
    public L2CoralScore(){
        addCommands(new L2coraly(), new PivotCommand(PivotState.L2CORALSCORE));
    }
}
