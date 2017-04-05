package org.usfirst.frc.team1923.robot.commands.drive;

import org.usfirst.frc.team1923.robot.Robot;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightWithGyroCommand extends Command {

    public double leftV = 6, rightV = 6;
    private PigeonImu gyro;
    private double head;

    // TODO tune values
    private final double P_CONST = 0.70;
    private final double CONTROLLER_BIAS = 6;
    private final double TOLERANCE = 0.100;
    private final double DISTANCE;
    private final double initialDistL;
    private final double initialDistR;
    private final double targetL;
    private final double targetR;

    public DriveStraightWithGyroCommand(double distance) {
        this.requires(Robot.driveSubSys);
        this.DISTANCE = distance;

        // Left target set
        initialDistL = Robot.driveSubSys.getLeftPosition();
        targetL = distance + initialDistL;

        // Right target set
        initialDistR = Robot.driveSubSys.getLeftPosition();
        targetR = distance + initialDistR;

        // timeout set
        this.setTimeout(Math.abs(distance) * 0.05 + 2);

        // initial heading set
        this.gyro = Robot.driveSubSys.getImu();
        this.head = this.gyro.GetFusedHeading(new FusionStatus());
        System.out.println("constructor");
        print();
    }

    @Override
    public void initialize() {
        // try {
        // Robot.driveSubSys.resetPosition();
        // Thread.sleep(250);
        // } catch (InterruptedException e) {
        // // TODO Auto-generated catch block
        // e.printStackTrace();
        // }
        // System.out.println("init Entered");
        // print();
        // System.out.println("init Exited");
        // print();
    }

    /*
     * The Proportional Algorithm for Gyro ============================= This
     * gyro Proportional-Only controller computes a output action every loop
     * sample time T as: Final Value = bias + C ∙ e, Where: bias = controller
     * bias or null value C = constant parameter e = controller error = SP - PV
     * (SP = set point, PV = measured process variable)
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
        System.out.println("Target Heading: " + this.head);
        System.out.println(" Heading: " + Robot.driveSubSys.getImu().GetFusedHeading(new FusionStatus()));
        System.out.println(" Left Error: " + (this.targetL - Robot.driveSubSys.getLeftPosition()));
        System.out.println(" Right Error: " + (this.targetR - Robot.driveSubSys.getRightPosition()));
        System.out.println(" Left Voltage: " + (this.leftV));
        System.out.println(" Right Voltage: " + (this.rightV));
        System.out.println(" Left Position: " + Robot.driveSubSys.getLeftPosition());
        System.out.println(" Right Position: " + Robot.driveSubSys.getRightPosition());
        System.out.println(" Distance target: " + this.DISTANCE);
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
        if (this.isTimedOut()) {
            System.out.println("Timed out");
            print();
            return true;
        }
        if (((Math.abs(Robot.driveSubSys.getLeftPosition() - this.targetL)) < Robot.driveSubSys.error)) {
            System.out.println("Left Error Condition triggered is Finished");
            System.out.println("Allowable Error: " + Robot.driveSubSys.error);
            print();
            return true;
        }

        if (((Math.abs(Robot.driveSubSys.getRightPosition() - this.targetR)) < Robot.driveSubSys.error)) {
            System.out.println("Right Error Condition triggered is Finished");
            System.out.println("Allowable Error: " + Robot.driveSubSys.error);
            print();
            return true;
        }
        return false;
    }

}
