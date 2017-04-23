package org.usfirst.frc.team1923.robot.commands.shooter;

import org.usfirst.frc.team1923.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ShiftShooterCommand extends InstantCommand {

    public ShiftShooterCommand() {
        Robot.shooterSubSys.toggle();
    }

    public ShiftShooterCommand(boolean up) {
        if (up)
            Robot.shooterSubSys.pistonUp();
        else Robot.shooterSubSys.pistonDown();
    }

}
