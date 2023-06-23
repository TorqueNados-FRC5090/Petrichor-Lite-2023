package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutonContainer {
    private Drivetrain drivetrain;
    private Arm arm;
    private Intake intake;

    public AutonContainer(Drivetrain drivetrain, Arm arm, Intake intake) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.intake = intake;
    }


    /** Auton that does nothing*/
    public Command doNothing() {
        return new SequentialCommandGroup(
            new GoToArmPreset(arm, ArmState.ZERO),
            new DoNothing(0, drivetrain)
        );
    }

    /** Shoots high forward and then moves backward out of the community. Spinning once it is out.*/
    public Command shootForwardAndDrive(){
        return new SequentialCommandGroup(
            new GoToArmPreset(arm, ArmState.FORWARD),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.shoot()),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.stop()),
            new DriveBackward(drivetrain, 14, .6),
            new DriveWithHeading(drivetrain, () -> 0, () -> 0, 180),    
            new InstantCommand(() -> drivetrain.resetHeading())        
        );
    }

    /** Shoots high forward. */
    public Command shootForward(){
        return new SequentialCommandGroup(
            new GoToArmPreset(arm, ArmState.FORWARD),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.shoot()),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.stop())
        );
    }

    /** Shoots high backward and then moves forward out of the community. */
    public Command shootBackwardAndDrive(){
        return new SequentialCommandGroup(
            new GoToArmPreset(arm, ArmState.BACKWARD),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.shoot()),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.stop()),
            new DriveBackward(drivetrain, -14, .6)
        );
    }
    
    /** Shoots high backward. */
    public Command shootBackward(){
        return new SequentialCommandGroup(
            new GoToArmPreset(arm, ArmState.BACKWARD),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.shoot()),
            new DoNothing(2, drivetrain),
            new InstantCommand(() -> intake.stop())
        );
    }    
}
