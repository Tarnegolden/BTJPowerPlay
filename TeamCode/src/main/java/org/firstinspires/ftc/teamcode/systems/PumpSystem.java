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

    private enum Direction {
        FORWARD, BACKWARD
    }

    private enum GrabbingState {
        GRABBING, DROPPING, IDLE
    }

    public PumpSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        stopPump = opMode.hardwareMap.get(TouchSensor.class, "stopPump");
        pumpR = opMode.hardwareMap.get(CRServo.class, "pr");
        pumpL = opMode.hardwareMap.get(CRServo.class, "pl");
        flipPump = opMode.hardwareMap.get(Servo.class, "rotatePump");
        pumpDCR = opMode.hardwareMap.get(Servo.class, "sideRight");
        pumpDCL = opMode.hardwareMap.get(Servo.class, "sideLeft");

        pumpR.setDirection(DcMotorSimple.Direction.REVERSE);
        setForward();
        stop();
    }

    public void setForward() {
        flipPump.setPosition(0);
        pumpDCR.setPosition(1);
        pumpDCL.setPosition(0);
        direction = Direction.FORWARD;
    }

    private void setBackward() {
        flipPump.setPosition(0.85);
        pumpDCR.setPosition(0);
        pumpDCL.setPosition(1);
        direction = Direction.BACKWARD;
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

    @Deprecated
    public boolean isPressed() {
        return stopPump.isPressed();
    }

    public void collect() {
        grab();
        while (opMode.opModeIsActive() && !stopPump.isPressed()) {}
        stop();
    }

    public void autoDrop() {
        ElapsedTime time = new ElapsedTime();
        drop();
        time.reset();
        time.startTime();
        while (opMode.opModeIsActive() && time.time() < 0.5) {}
        stop();
    }
}
