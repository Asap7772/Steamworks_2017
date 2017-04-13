package org.usfirst.frc.team1923.robot.commands.drive;

import org.usfirst.frc.team1923.robot.Robot;
import org.usfirst.frc.team1923.robot.subsystems.DrivetrainSubsystem;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightWithGyroCommandFormatted extends Command {

    public double leftV = 6, rightV = 6;
    private PigeonImu gyro;
    private double head;

    private final double P_CONST = 0.70;
    private final double CONTROLLER_BIAS = 6;
    private final double TOLERANCE = 0.100;
    // DISTANCES below in rotations
    private final double DISTANCE;
    private final double TARGET_LEFT;
    private final double TARGET_RIGHT;

    public DriveStraightWithGyroCommandFormatted(double distance) {
        this.requires(Robot.driveSubSys);
        this.DISTANCE = DrivetrainSubsystem.distanceToRotation(distance);

        // Left target set (current enc position + distance)
        TARGET_LEFT = DISTANCE + Robot.driveSubSys.getLeftPosition();

        // Right target set (current enc position + distance)
        TARGET_RIGHT = DISTANCE + Robot.driveSubSys.getLeftPosition();

        // timeout set
        this.setTimeout(Math.abs(distance) * 0.05 + 2);

        // initial heading set
        this.gyro = Robot.driveSubSys.getImu();
        this.head = this.gyro.GetFusedHeading(new FusionStatus());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        print();
        // error calculated
        double error = head - this.gyro.GetFusedHeading(new FusionStatus()) % 360;

        if (Math.abs(error) > this.TOLERANCE) {
            // TODO Consider changing both right and left voltage
            rightV = CONTROLLER_BIAS + P_CONST * error;
            // keeps range of right voltage between [-12, 12]
            rightV = Math.min(12, rightV);
            rightV = Math.max(-12, rightV);
        }
        Robot.driveSubSys.drive(leftV, rightV, TalonControlMode.Voltage);
    }

    public void print() {
        System.out.println("Target Heading: " + this.head);
        System.out.println(" Heading: " + Robot.driveSubSys.getImu().GetFusedHeading(new FusionStatus()));
        System.out.println(" Left Error: " + (this.TARGET_LEFT - Robot.driveSubSys.getLeftPosition()));
        System.out.println(" Right Error: " + (this.TARGET_RIGHT - Robot.driveSubSys.getRightPosition()));
        System.out.println(" Left Voltage: " + (this.leftV));
        System.out.println(" Right Voltage: " + (this.rightV));
        System.out.println(" Left Position: " + Robot.driveSubSys.getLeftPosition());
        System.out.println(" Right Position: " + Robot.driveSubSys.getRightPosition());
        System.out.println(" Distance target (rotations): " + this.DISTANCE);
        System.out.println();
    }

    @Override
    public void end() {
        Robot.driveSubSys.drive(0, 0, TalonControlMode.PercentVbus);
    }

    @Override
    public void interrupted() {
        end();
    }

    @Override
    protected boolean isFinished() {
        if (this.isTimedOut()) {
            System.out.println("Timed out");
            print();
            return true;
        }
        if (((Math.abs(Robot.driveSubSys.getLeftPosition() - this.TARGET_LEFT)) < Robot.driveSubSys.ALLOWABLE_ERROR)) {
            System.out.println("Left Error Condition triggered is Finished");
            System.out.println("Allowable Error: " + Robot.driveSubSys.error);
            print();
            return true;
        }

        if (((Math.abs(Robot.driveSubSys.getRightPosition() - this.TARGET_RIGHT)) < Robot.driveSubSys.ALLOWABLE_ERROR)) {
            System.out.println("Right Error Condition triggered is Finished");
            System.out.println("Allowable Error: " + Robot.driveSubSys.error);
            print();
            return true;
        }
        return false;
    }

}
