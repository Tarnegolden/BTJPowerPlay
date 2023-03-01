package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PumpSystem {
    private final TouchSensor stopPump;
    private final CRServo pumpR;
    private final CRServo pumpL;
    private final Servo flipPump;
    private final Servo pumpDCR;
    private final Servo pumpDCL;
    private Direction direction;
    private GrabbingState grabbingState;
    private LinearOpMode opMode;
    private ElevatorSystem elevatorSystem;

    private enum Direction {
        FORWARD, BACKWARD
    }

    private enum GrabbingState {
        GRABBING, DROPPING, IDLE
    }

    public PumpSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.elevatorSystem = new ElevatorSystem(opMode);
        stopPump = opMode.hardwareMap.get(TouchSensor.class, "stopPump");
        pumpR = opMode.hardwareMap.get(CRServo.class, "pr");
        pumpL = opMode.hardwareMap.get(CRServo.class, "pl");
        flipPump = opMode.hardwareMap.get(Servo.class, "rotatePump");
        pumpDCR = opMode.hardwareMap.get(Servo.class, "sideRight");
        pumpDCL = opMode.hardwareMap.get(Servo.class, "sideLeft");

        pumpR.setDirection(DcMotorSimple.Direction.REVERSE);
//        setForward();
        direction = Direction.FORWARD;
        stop();
    }

    public PumpSystem(LinearOpMode opMode, ElevatorSystem elevatorSystem) {
        this.opMode = opMode;
        this.elevatorSystem = elevatorSystem;
        stopPump = opMode.hardwareMap.get(TouchSensor.class, "stopPump");
        pumpR = opMode.hardwareMap.get(CRServo.class, "pr");
        pumpL = opMode.hardwareMap.get(CRServo.class, "pl");
        flipPump = opMode.hardwareMap.get(Servo.class, "rotatePump");
        pumpDCR = opMode.hardwareMap.get(Servo.class, "sideRight");
        pumpDCL = opMode.hardwareMap.get(Servo.class, "sideLeft");

        pumpR.setDirection(DcMotorSimple.Direction.REVERSE);
//        setForward();
        direction = Direction.FORWARD;
        stop();
    }

    public void setForward() {
        new Thread(() -> {
            flipPump.setPosition(0.85);
            opMode.sleep(350);
            pumpDCR.setPosition(1);
            pumpDCL.setPosition(0);
            direction = Direction.FORWARD;
        }).start();
    }

    public void setBackward() {
        new Thread(() -> {
            flipPump.setPosition(0);
            opMode.sleep(350);
            pumpDCR.setPosition(0);
            pumpDCL.setPosition(1);
            direction = Direction.BACKWARD;
        }).start();
    }

    public void flip() {
        if (direction == Direction.BACKWARD) {
            setForward();
        } else {
            setBackward();
        }
    }

    public void stop() {
        pumpR.setPower(0);
        pumpL.setPower(0);
        grabbingState = GrabbingState.IDLE;
    }

    public void grab() {
        pumpR.setPower(1);
        pumpL.setPower(1);
        grabbingState = GrabbingState.GRABBING;
    }

    public void grabToggle() {
        if (grabbingState == GrabbingState.IDLE) {
            grab();
        } else {
            stop();
        }
    }

    public void drop() {
        pumpR.setPower(-1);
        pumpL.setPower(-1);
        grabbingState = GrabbingState.DROPPING;
    }

    public void dropToggle() {
        if (grabbingState == GrabbingState.IDLE) {
            drop();
        } else {
            stop();
        }
    }

    public boolean isPressed() {
        return stopPump.isPressed();
    }

    public void collect() {
        new Thread(() -> {
//            while (opMode.opModeIsActive() && elevatorSystem.getCurrentPosition() != elevatorSystem.getTargetPosition()) {}
            ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            grab();
            time.reset();
            time.startTime();
//        while (opMode.opModeIsActive() && !isPressed()) {}
            while (opMode.opModeIsActive() && time.time() < 500) {}
            stop();
        }).start();
    }

    public void autoDrop() {
        new Thread(() -> {
//            while (opMode.opModeIsActive() && elevatorSystem.getCurrentPosition() != elevatorSystem.getTargetPosition()) {}
            ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            drop();
            time.reset();
            time.startTime();
            while (opMode.opModeIsActive() && time.time() < 500) {}
            stop();
        }).start();
    }
}
