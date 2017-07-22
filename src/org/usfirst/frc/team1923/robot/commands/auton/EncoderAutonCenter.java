package org.usfirst.frc.team1923.robot.commands.auton;

import org.usfirst.frc.team1923.robot.RobotMap;
import org.usfirst.frc.team1923.robot.commands.drive.DriveMotionMagicCommand;
import org.usfirst.frc.team1923.robot.commands.gear.GearCommand;
import org.usfirst.frc.team1923.robot.commands.gear.SlideCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class EncoderAutonCenter extends CommandGroup {

    public EncoderAutonCenter() {
        this.addSequential(new DriveMotionMagicCommand(RobotMap.CENTER_AUTON_DISTANCE));
        this.addSequential(new SlideCommand());
        this.addSequential(new GearCommand());
    }

}
