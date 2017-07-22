package org.usfirst.frc.team1923.robot.commands.shooter;

import org.usfirst.frc.team1923.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ShooterCommand extends Command {

    int speed;

    public ShooterCommand(int speed) {
        this(speed, 10);
    }

    public ShooterCommand(int speed, double timeout) {
        this.setTimeout(timeout);
        this.speed = speed;
    }

    @Override
    public void execute() {
        Robot.shooterSubSys.setFlywheelSpeed(speed);
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void interrupted() {

    }

}
