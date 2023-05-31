package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.subsystems.Arm;

public class AutonContainer {
    private Drivetrain drivetrain;
    private Arm arm;

    public AutonContainer(Drivetrain drivetrain, Arm arm) {
        this.drivetrain = drivetrain;
        this.arm = arm;
    }


    /** Auton that drops a piece high, reverses, and sets heading*/
    public Command dropHigh() {
        return new SequentialCommandGroup(
            new GoToArmPreset(arm, ArmState.ZERO),
            new DoNothing(0, drivetrain)
        );
    }
}
