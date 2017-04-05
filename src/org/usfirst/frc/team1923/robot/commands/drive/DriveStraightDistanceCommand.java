package org.usfirst.frc.team1923.robot.commands.drive;

import org.usfirst.frc.team1923.robot.Robot;
import org.usfirst.frc.team1923.robot.utils.PIDController;

import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightDistanceCommand extends Command {

    public double leftV = 6, rightV = 6;

    // TODO tune values
    private final double P_CONST = 0.70;
    private final double I_CONST = 0;
    private final double D_CONST = 0;
    private final double I_ZONE = 5;
    private final double CONTROLLER_BIAS = 6;
    private final double TOLERANCE = 0.100;
    private final double initialDistL;
    private final double initialDistR;
    private final double targetL;
    private final double targetR;

    private ImuTarget target;
    private Drivetrain output;
    private PIDController controller;

    public DriveStraightDistanceCommand(double distance) {
        this.requires(Robot.driveSubSys);

        // initialized PID Controller
        this.target = new ImuTarget();
        this.output = new Drivetrain();
        this.controller = new PIDController(P_CONST, I_CONST, D_CONST, this.target, this.output);
        this.controller.setContinuous(true);
        this.controller.setAbsoluteTolerance(TOLERANCE);
        this.controller.setOutputRange(-6, 6);
        this.controller.setInputRange(-360, 360);
        this.controller.setSetpoint(Robot.driveSubSys.getImu().GetFusedHeading(new FusionStatus()));
        this.controller.setIZone(I_ZONE);

        // Left target set
        initialDistL = Robot.driveSubSys.getLeftPosition();
        targetL = distance + initialDistL;

        // Right target set
        initialDistR = Robot.driveSubSys.getLeftPosition();
        targetR = distance + initialDistR;

        // timeout set
        this.setTimeout(Math.abs(distance) * 0.05 + 2);

        System.out.println("Constructor Exited");
        print();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    public void print() {
    }

    @Override
    public void end() {
        Robot.driveSubSys.drive(0, 0, TalonControlMode.PercentVbus);
    }

    @Override
    public void interrupted() {
        System.out.println("interupted");
        end();
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut() || (Math.abs(Robot.driveSubSys.getLeftPosition() - this.targetL)) < Robot.driveSubSys.error
                || (Math.abs(Robot.driveSubSys.getRightPosition() - this.targetR)) < Robot.driveSubSys.error;
    }

    public class ImuTarget implements PIDSource {

        private PigeonImu imu;
        private double startingAngle;

        public ImuTarget() {
            this.imu = Robot.driveSubSys.getImu();
            this.startingAngle = this.getHeading();
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {
            return startingAngle - this.getHeading();
        }

        protected double getHeading() {
            return this.imu.GetFusedHeading(new FusionStatus());
        }

    }

    public class Drivetrain implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            Robot.driveSubSys.drive(leftV, CONTROLLER_BIAS + output, TalonControlMode.Voltage);
        }

    }

}
