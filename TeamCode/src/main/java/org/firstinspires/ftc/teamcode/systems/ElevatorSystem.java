package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ElevatorSystem {
    private final DcMotor elevatorRight;
    private final DcMotor elevatorLeft;

    public enum Level {
        GROUND, LOW, MEDIUM, HIGH
    }

    public ElevatorSystem(LinearOpMode opMode) {
        elevatorRight = opMode.hardwareMap.get(DcMotor.class, "ER");
        elevatorLeft = opMode.hardwareMap.get(DcMotor.class, "EL");

        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goTo(int ticks) {
        elevatorRight.setTargetPosition(ticks);
        elevatorLeft.setTargetPosition(ticks);

        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevatorRight.setPower(1);
        elevatorLeft.setPower(1);
    }

    public void goTo(Level level) {
        switch (level) {
            case HIGH:
                goTo(2900);
                break;
            case MEDIUM:
                goTo(2150);
                break;
            case LOW:
                goTo(1350);
                break;
            case GROUND:
                goTo(0);
        }
    }

    public int getCurrentPosition() {
        return elevatorLeft.getCurrentPosition();
    }

    public int getTargetPosition() {
        return elevatorLeft.getTargetPosition();
    }
}
