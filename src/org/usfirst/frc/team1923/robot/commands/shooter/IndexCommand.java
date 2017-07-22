package org.usfirst.frc.team1923.robot.commands.shooter;

import org.usfirst.frc.team1923.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class IndexCommand extends Command {

    double voltage;

    public IndexCommand(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void execute() {

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.shooterSubSys.setIndexerVoltage(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
