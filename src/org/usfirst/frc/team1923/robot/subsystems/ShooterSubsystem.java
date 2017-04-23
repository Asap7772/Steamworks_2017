package org.usfirst.frc.team1923.robot.subsystems;

import org.usfirst.frc.team1923.robot.RobotMap;
import org.usfirst.frc.team1923.robot.commands.shooter.IntakeState;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShooterSubsystem extends Subsystem {

    // Talon that controls shooter motor
    private CANTalon shooterIntake;
    private IntakeState currState;

    // Talon that controls shooter piston
    private DoubleSolenoid shooterPiston;
    private boolean pistonUp;

    // constructor initializes the talon, the solenoid and supporting variables
    public ShooterSubsystem() {
        shooterIntake = new CANTalon(RobotMap.SHOOTER_PORT); // defaulted on
                                                             // Percent V-Bus

        currState = IntakeState.NEUTRAL;
        shooterPiston = new DoubleSolenoid(RobotMap.PCM_MODULE_NUM, RobotMap.SHOOTER_FORWARD_PORT, RobotMap.SHOOTER_BACKWARD_PORT);
        pistonUp = false;
    }

    public void intake() {
        currState = IntakeState.INTAKE;
        shooterIntake.set(-1);
    }

    public void neutral() {
        currState = IntakeState.NEUTRAL;
        shooterIntake.set(0);
    }

    public void outake() {
        currState = IntakeState.OUTAKE;
        shooterIntake.set(1);
    }

    public void toggle() {
        if (pistonUp)
            pistonDown();
        else pistonUp();
    }

    public void pistonUp() {
        if (!pistonUp) {
            shooterPiston.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void pistonDown() {
        if (pistonUp) {
            shooterPiston.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public IntakeState getIntakeState() {
        return currState;
    }

    public String getPistonState() {
        return pistonUp ? "Up" : "Down";
    }

    @Override
    protected void initDefaultCommand() {

    }

}
