package org.usfirst.frc.team1923.robot.commands.drive;

import org.usfirst.frc.team1923.robot.Robot;
import org.usfirst.frc.team1923.robot.RobotMap;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightWithGyroCommand extends Command {

    public double leftV = 6, rightV = 6;
    private PigeonImu gyro;
    private double head;

    private final double P_CONST = 0.070;
    private final double CONTROLLER_BIAS = 0;
    private final double TOLERANCE = 0.100;

    public DriveStraightWithGyroCommand(double distance) {
        this.requires(Robot.driveSubSys);
        Robot.driveSubSys.setDistance(distance);
        this.setTimeout(Math.abs(distance) * 0.05 + 2);
        this.gyro = Robot.driveSubSys.getImu();
        // initial heading
        this.head = 0.0;
        this.gyro.SetFusedHeading(head);
    }

    @Override
    public void initialize() {

    }

    /* The P-Only Algorithm
     *  ==================== 
     *  The P-Only controller computes a CO action every loop sample time T as: CO = CObias + Kc∙e(t), Where:
     *  CObias = controller bias or null value
     *  Kc = controller gain, a tuning parameter 
     *  e(t) = controller error = SP – PV (SP = set point, PV = measured process variable)
     */

    // if (error > 0)
    // leftV *= (1 + this.P_CONST);
    // else leftV *= (this.P_CONST);
    protected void pHeading() {
        double process = this.gyro.GetFusedHeading(new FusionStatus()) % 360;
        double error = head - process;
        if (Math.abs(error) > this.TOLERANCE) {
            leftV = this.CONTROLLER_BIAS + this.P_CONST * error;
            //keeps it in [-12, 12]
            if (leftV > 12)
                leftV = 12;
            else if (leftV < -12)
                leftV = -12;
        }
        Robot.driveSubSys.drive(leftV, rightV, TalonControlMode.Voltage);
    }

    @Override
    public void execute() {
        this.pHeading();
        if (RobotMap.DEBUG) {
            System.out.println(System.currentTimeMillis() + "Heading: " + Robot.driveSubSys.getImu().GetFusedHeading(new FusionStatus()));
            System.out.println(System.currentTimeMillis() + "Left Error: " + Robot.driveSubSys.getManualErrorLeft());
            System.out.println(System.currentTimeMillis() + "Right Error: " + Robot.driveSubSys.getManualErrorRight());
        }
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
        return isTimedOut() || ((Math.abs(Robot.driveSubSys.getManualErrorLeft())) < Robot.driveSubSys.ALLOWABLE_ERROR)
                && (Math.abs(Robot.driveSubSys.getManualErrorRight()) < Robot.driveSubSys.ALLOWABLE_ERROR);
    }

}
