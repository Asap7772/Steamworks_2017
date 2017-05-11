package org.usfirst.frc.team1923.robot.commands;

import org.usfirst.frc.team1923.robot.Robot;

public class SrishtiCommand extends TimedCommand {

    private final boolean srishtiLove = true;

    public ControllerRumbleCommand( {
      
    }

    @Override
    protected void interrupted() {
        dontEnd();
    }

}
