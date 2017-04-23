package org.usfirst.frc.team1923.robot.commands.shooter;

import org.usfirst.frc.team1923.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeShooterCommand extends Command {

    IntakeState state;

    public IntakeShooterCommand() {
        this(IntakeState.NEUTRAL);
    }

    public IntakeShooterCommand(IntakeState state) {
        this.state = state;
    }

    public IntakeShooterCommand(IntakeState state, double timeout) {
        this.state = state;
        this.setTimeout(timeout);
    }

    @Override
    protected void execute() {
        switch (state) {
            case INTAKE:
                Robot.shooterSubSys.intake();
                break;
            case OUTAKE:
                Robot.shooterSubSys.outake();
                break;
            default:
                Robot.shooterSubSys.neutral();
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return state.equals(IntakeState.NEUTRAL) || isTimedOut();
    }

    protected void interupted() {
        end();
    }

    @Override
    protected void end() {
        Robot.shooterSubSys.neutral();
    }
}
