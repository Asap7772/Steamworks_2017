package org.usfirst.frc.team1923.robot.subsystems;

import org.usfirst.frc.team1923.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends Subsystem {

    // Talon that controls shooter motor
    private CANTalon flyWheel;
    private CANTalon indexer;

    private final double P_CONSTANT = 0;
    private final double I_CONSTANT = 0;
    private final double D_CONSTANT = 0;
    private final double FEED_FORWARD = 0.4;
    private final int tolerance = 1;

    // constructor initializes the talon, the solenoid and supporting variables
    public ShooterSubsystem() {
        flyWheel = new CANTalon(RobotMap.SHOOTER_PORT);
        indexer = new CANTalon(RobotMap.INDEXER_PORT);
        flyWheel.changeControlMode(TalonControlMode.Speed);
        flyWheel.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        flyWheel.configPeakOutputVoltage(12, -12);
        flyWheel.configNominalOutputVoltage(0, 0);
        flyWheel.setPID(P_CONSTANT, I_CONSTANT, D_CONSTANT);
        flyWheel.setF(FEED_FORWARD);
        flyWheel.setAllowableClosedLoopErr(tolerance);
        flyWheel.reverseOutput(false);
        flyWheel.reverseSensor(false);
    }

    public void setIndexerVoltage(double voltage) {
        indexer.set(voltage);
        SmartDashboard.putNumber("Indexer Voltage", indexer.getOutputVoltage());
    }

    public void setFlywheelSpeed(double speed) {
        flyWheel.set(speed);
        SmartDashboard.putNumber("Encode Velocity", flyWheel.getSpeed());
    }

    @Override
    protected void initDefaultCommand() {

    }

}
