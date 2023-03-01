package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DetectionSystem;
import org.firstinspires.ftc.teamcode.systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.systems.PumpSystem;
import org.firstinspires.ftc.teamcode.systems.Systems;
import org.firstinspires.ftc.teamcode.systems.Trajectories;

@TeleOp(name = "Pump Test", group = "Test")
public class PumpTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        PumpSystem pumpSystem = new PumpSystem(this);

        boolean triangle = false;
        boolean circle = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y && !triangle) {
                pumpSystem.setForward();
            }
            if (gamepad1.b && !circle) {
                pumpSystem.setBackward();
            }
            telemetry.addData("Sensor Pressed: ", pumpSystem.isPressed());
            telemetry.update();
            triangle = gamepad1.triangle;
            circle = gamepad1.circle;
        }
    }
}
