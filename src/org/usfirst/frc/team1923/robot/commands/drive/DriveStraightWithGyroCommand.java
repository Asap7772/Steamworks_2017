package org.usfirst.frc.team1923.robot.commands.drive;

import org.usfirst.frc.team1923.robot.Robot;
import org.usfirst.frc.team1923.robot.subsystems.DrivetrainSubsystem;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightWithGyroCommand extends Command {

    public double leftV = 8, rightV = 8;
    private PigeonImu gyro;
    private double head;

    // TODO tune values
    private final double P_CONST = 0.070;
    private final double CONTROLLER_BIAS = 8;
    private final double TOLERANCE = 0.100;
    private final double DISTANCE;

    public DriveStraightWithGyroCommand(double distance) {
        this.requires(Robot.driveSubSys);
        this.DISTANCE = distance;
        this.setTimeout(Math.abs(distance) * 0.05 + 2);
        this.gyro = Robot.driveSubSys.getImu();
        // initial heading set
        this.head = this.gyro.GetFusedHeading(new FusionStatus());
        System.out.println("constructor");
        print();
    }

    @Override
    public void initialize() {
        System.out.println("init");
        print();
    }

    /*
     * The Proportional Algorithm for Gyro ============================= This
     * gyro Proportional-Only controller computes a output action every loop
     * sample time T as: Final Value = bias + C ∙ e, Where: bias = controller
     * bias or null value C = constant parameter e = controller error = SP - PV
     * (SP = set point, PV = measured process variable)
     */

    protected void pHeading() {
        double process = this.gyro.GetFusedHeading(new FusionStatus()) % 360;
        double error = head - process;
        if (Math.abs(error) > this.TOLERANCE) {
            // Updates left voltage value
            // will allow me to change the vol tage based on error
            leftV = CONTROLLER_BIAS + P_CONST * error;

            // Keeps Left Voltage in the range: [-12, 12]
            leftV = Math.min(12, leftV);
            leftV = Math.max(-12, leftV);
        }
    }

    @Override
    public void execute() {
        this.pHeading();
        Robot.driveSubSys.drive(leftV, rightV, TalonControlMode.Voltage);
        print();
    }

    public void print() {
        double dist = DrivetrainSubsystem.distanceToRotation(this.DISTANCE);
        System.out.println(" Heading: " + Robot.driveSubSys.getImu().GetFusedHeading(new FusionStatus()));
        System.out.println(" Left Error: " + (Robot.driveSubSys.getLeftPosition() - dist));
        System.out.println(" Right Error: " + (Robot.driveSubSys.getRightPosition() - dist));
        System.out.println(" Left Voltage: " + (this.leftV));
        System.out.println(" Right Voltage: " + (this.rightV));
    }

    @Override
    public void end() {
        Robot.driveSubSys.drive(0, 0, TalonControlMode.PercentVbus);
        System.out.println("end");
        print();
    }

    @Override
    public void interrupted() {
        System.out.println("interupted");
        end();
    }

    @Override
    protected boolean isFinished() {
        double dist = DrivetrainSubsystem.distanceToRotation(this.DISTANCE);
        return ((Math.abs(Robot.driveSubSys.getLeftPosition() - dist)) < Robot.driveSubSys.ALLOWABLE_ERROR)
                || (Math.abs(Robot.driveSubSys.getRightPosition() - dist) < Robot.driveSubSys.ALLOWABLE_ERROR);
    }

}
