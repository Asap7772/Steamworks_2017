package org.usfirst.frc.team1923.robot.commands.drive;

import org.usfirst.frc.team1923.robot.Robot;
import org.usfirst.frc.team1923.robot.utils.PIDController;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightWithGyroCommand extends Command {

    public double leftV, rightV;
    private PIDController controller;
    private PigeonImu gyro;
    private FusionStatus fusionStatus;
    private double head;

    private final double P_CONST = 0.070;
    private final double TOLERANCE = 0.100;

    public DriveStraightWithGyroCommand(double distance) {
        this.requires(Robot.driveSubSys);
        Robot.driveSubSys.setDistance(distance);
        this.setTimeout(Math.abs(distance) * 0.05 + 2);
        leftV = 6;
        rightV = 6;
        this.gyro = Robot.driveSubSys.getImu();
        this.fusionStatus = new FusionStatus();
        this.head = this.gyro.GetFusedHeading(this.fusionStatus) % 360;
    }

    @Override
    public void initialize() {

    }

    protected double getHeading() {
        return this.gyro.GetFusedHeading(this.fusionStatus) % 360;
    }

    protected double getGyroError() {
        return head - getHeading();
    }

    protected void adjustHeading() {
        if (getGyroError() > 0) {
            leftV *= (1 + this.P_CONST);
        } else {
            leftV *= (this.P_CONST);
        }
        if (leftV > 12)
            leftV = 12;
        else if (leftV < -12)
            leftV = -12;
    }

    @Override
    public void execute() {
        if (getGyroError() > this.TOLERANCE)
            adjustHeading();
        Robot.driveSubSys.drive(leftV, rightV, TalonControlMode.Voltage);
        System.out.println(System.currentTimeMillis() + "Left Error: " + Robot.driveSubSys.getManualErrorLeft());
        System.out.println(System.currentTimeMillis() + "Right Error: " + Robot.driveSubSys.getManualErrorRight());
    }

    @Override
    public void end() {
        Robot.driveSubSys.drive(0, 0, TalonControlMode.PercentVbus);
    }

    @Override
    public void interrupted() {

    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || ((Math.abs(Robot.driveSubSys.getManualErrorLeft())) < Robot.driveSubSys.ALLOWABLE_ERROR)
                && (Math.abs(Robot.driveSubSys.getManualErrorRight()) < Robot.driveSubSys.ALLOWABLE_ERROR);
    }

}
