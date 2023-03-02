package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Systems;
import org.firstinspires.ftc.teamcode.systems.Trajectories;

@TeleOp(name = "Sample Auto", group = "Test")
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Systems systems = new Systems(this, drive);
        Trajectories trajectories = new Trajectories(drive, systems);

        waitForStart();

        if (isStopRequested()) return;

        trajectories.BL();
        telemetry.addData("Time: ", time);
        telemetry.update();

        while (opModeIsActive());
    }
}
