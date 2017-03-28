package org.usfirst.frc.team1923.robot.commands.auton;

import org.usfirst.frc.team1923.robot.RobotMap;
import org.usfirst.frc.team1923.robot.commands.driveCommands.DriveDistanceCommand;
import org.usfirst.frc.team1923.robot.commands.driveCommands.GyroTurnCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class BlueHopperCommand extends CommandGroup{
	public BlueHopperCommand(){
		this.addSequential(new DriveDistanceCommand(RobotMap.BASELINE_DISTANCE + RobotMap.HOPPER_WIDTH/2));
		this.addSequential(new GyroTurnCommand(90));
		this.addSequential(new DriveDistanceCommand(RobotMap.FORWARD_DISTANCE_HOPPER));
		this.addSequential(new GyroTurnCommand(-90));
		this.addSequential(new DriveDistanceCommand(RobotMap.NEUTRAL_ZONE_LENGTH));
	}
}