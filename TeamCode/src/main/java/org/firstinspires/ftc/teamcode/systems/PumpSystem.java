package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    private final LinearOpMode opMode;
    private final ElevatorSystem elevatorSystem;
    private final Systems systems;

    private enum Direction {
        FORWARD, BACKWARD
    }

    private enum GrabbingState {
        GRABBING, DROPPING, IDLE
    }

    public PumpSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.elevatorSystem = new ElevatorSystem(opMode);
        this.systems = new Systems(opMode);
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

    public PumpSystem(LinearOpMode opMode, Systems systems) {
        this.opMode = opMode;
        this.elevatorSystem = systems.elevatorSystem;
        this.systems = systems;
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
        grab();
//        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        time.reset();
//        time.startTime();
//        while (opMode.opModeIsActive() && time.time() < 500) {
//            systems.drive.setDrivePower(new Pose2d(0, 0.2));
//        }
        while (opMode.opModeIsActive() && !isPressed()) {
            systems.drive.setDrivePower(new Pose2d(0, 0.2));
        }
        systems.drive.setDrivePower(new Pose2d());
        stop();
    }

    public void autoDrop() {
        new Thread(() -> {
//            while (opMode.opModeIsActive() && elevatorSystem.getCurrentPosition() != elevatorSystem.getTargetPosition()) {}
            ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            drop();
            time.reset();
            time.startTime();
            while (opMode.opModeIsActive() && time.time() < 500) {
            }
            stop();
        }).start();
    }
}
