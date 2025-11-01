package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake extends SubsystemBase{

//    private final Robot robot = Robot.getInstance();
    MotorEx motor;

    public enum MotorState {
        REVERSE,
        STOP,
        FORWARD,
        PASSIVE
    }

    public static double INTAKE_FORWARD_SPEED = 0.5;
    public static double INTAKE_REVERSE_SPEED = -0.5; // unused
    public static double INTAKE_PASSIVE_SPEED = 0.2;
    public MotorState motorState = MotorState.STOP;

    public Intake(MotorEx intakeMotor) {
        motor = intakeMotor;
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorState(MotorState motorState) {
        this.motorState = motorState;
    }

    public void setIntakeState() {
        switch (motorState) {
            case FORWARD:
                motor.set(INTAKE_FORWARD_SPEED);
                break;
            case REVERSE:
                motor.set(INTAKE_REVERSE_SPEED);
                break;
            case PASSIVE:
                motor.set(INTAKE_PASSIVE_SPEED);
                break;
            case STOP:
            default:
                motor.set(0.0);
                break;
        }
    }


    public void toggleIntake() {
        if (motorState.equals(MotorState.FORWARD)) {
            setMotorState(MotorState.STOP);
        } else {
            setMotorState(MotorState.FORWARD);
        }
        setIntakeState();
    }


    @Override
    public void periodic() {
        setIntakeState();
    }
}


