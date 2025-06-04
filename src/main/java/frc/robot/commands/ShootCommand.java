package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    private ShooterSubsystem s_ShooterSubsystem;
    public ShootCommand(ShooterSubsystem s_ShooterSubsystem){
        addRequirements(s_ShooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        new SequentialCommandGroup(
            new InstantCommand(() -> s_ShooterSubsystem.set(1)),
            new WaitCommand(2),
            new InstantCommand(() -> s_ShooterSubsystem.set(0))            
        );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
    return false;
    }

}
