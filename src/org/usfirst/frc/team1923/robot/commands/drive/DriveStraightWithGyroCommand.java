package org.usfirst.frc.team1923.robot.commands.drive;

import org.usfirst.frc.team1923.robot.Robot;
import org.usfirst.frc.team1923.robot.subsystems.DrivetrainSubsystem;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightWithGyroCommand extends Command {

    public double leftV = 6, rightV = 6;
    private PigeonImu gyro;
    private double head;

    // TODO tune values
    private final double P_CONST = 0.070;
    private final double CONTROLLER_BIAS = 6;
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
        System.out.println("init Entered");
        print();
        Robot.driveSubSys.resetPosition();
        print();
        System.out.println("init Exited");
    }

    /*
     * The Proportional Algorithm for Gyro
     *  ============================= 
     *  This gyro Proportional-Only controller computes a output action every loop sample time T as:
     *   Final Value = bias + C ∙ e, Where: 
     *   bias = controller bias or null value 
     *   C = constant parameter 
     *   e = controller error = SP - PV (SP = set point, PV = measured process variable)
     */

    protected void pHeading() {
        System.out.println("PHeading Entered");
        print();
        double error = head - this.gyro.GetFusedHeading(new FusionStatus()) % 360;
        System.out.print("Target Heading" + this.head);
        System.out.print(" Current Heading: " + this.gyro.GetFusedHeading(new FusionStatus()) % 360);
        System.out.println(" Error: " + error);
        if (Math.abs(error) > this.TOLERANCE) {
            // Updates right voltage value
            // will allow me to change the voltage based on error
            rightV = CONTROLLER_BIAS + P_CONST * error;

            System.out.println("RightV initial: " + this.rightV);
            // Keeps Right Voltage in the range: [-12, 12]
            rightV = Math.min(12, rightV);
            rightV = Math.max(-12, rightV);
            System.out.println("RightV changed: " + this.rightV);
        }
        System.out.println("PHeading Exited");
        print();
    }

    @Override
    public void execute() {
        System.out.println("Execute Entered");
        print();
        this.pHeading();
        Robot.driveSubSys.drive(leftV, rightV, TalonControlMode.Voltage);
        System.out.println("Execute Exited");
    }

    public void print() {
        double dist = DrivetrainSubsystem.distanceToRotation(this.DISTANCE);
        System.out.print("Target Heading: " + this.head); 
        System.out.print(" Heading: " + Robot.driveSubSys.getImu().GetFusedHeading(new FusionStatus()));
        System.out.print(" Left Error: " + (Robot.driveSubSys.getLeftPosition() - dist));
        System.out.print(" Right Error: " + (Robot.driveSubSys.getRightPosition() - dist));
        System.out.print(" Left Voltage: " + (this.leftV));
        System.out.print(" Right Voltage: " + (this.rightV));
        System.out.print(" Left Position: " + Robot.driveSubSys.getLeftPosition());
        System.out.print(" Right Position: " + Robot.driveSubSys.getRightPosition());
        System.out.print(" Distance target: " + this.DISTANCE);
        System.out.println();
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
        print();
        end();
    }

    @Override
    protected boolean isFinished() {
        double dist = DrivetrainSubsystem.distanceToRotation(this.DISTANCE);
        if (this.isTimedOut()) {
            System.out.println("Timed out");
            print();
            return true;
        }
        if (((Math.abs(Robot.driveSubSys.getLeftPosition() - dist)) < Robot.driveSubSys.ALLOWABLE_ERROR)) {
            System.out.println("Left Error Condition triggered is Finished");
            System.out.println("Allowable Error: " + Robot.driveSubSys.ALLOWABLE_ERROR);
            print();
            return true;
        }

        if (((Math.abs(Robot.driveSubSys.getRightPosition() - dist)) < Robot.driveSubSys.ALLOWABLE_ERROR)) {
            System.out.println("Right Error Condition triggered is Finished");
            System.out.println("Allowable Error: " + Robot.driveSubSys.ALLOWABLE_ERROR);
            print();
            return true;
        }
        return false;
    }

}
