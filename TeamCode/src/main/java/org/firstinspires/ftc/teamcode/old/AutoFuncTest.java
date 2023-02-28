package org.firstinspires.ftc.teamcode.old;

//import com.google.blocks.ftcrobotcontroller.runtime.DcMotorAccess;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name = "AutoFuncTest", group = "BLUE", preselectTeleOp = "Controller")
public class AutoFuncTest extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    DcMotor motorFrontLeft;

    Servo turnPump;
    Servo PumpDirectionChangerRight;
    Servo PumpDirectionChangerLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    Servo stickHolder;
    CRServo PUMPR, PUMPL;

    int tiles = 1200;

    final int[] ConeStackHight = new int[]{0, 100, 200, 300, 400, 500};
    int coneInStack = 5;

    DcMotor elevatorRight;
    DcMotor elevatorLeft;

    int[][] junctionHight = new int[][]{
            {0, 1350, 0, 1350, 0},
            {1350, 2150, 2900, 2150, 1350},
            {0, 2900, 0, 2900, 0},
            {1350, 2150, 2900, 2150, 1350},
            {0, 1350, 0, 1350, 0}};
    int[] coneStackHight = new int[]{100, 200, 300, 400};
    int currentConeInStack = 3;

    ColorSensor sleeveSensor;
    ColorSensor tapeSensor;

    boolean firstJunction = true;

    public ElapsedTime runTime = new ElapsedTime();

    String sleeveColor;

    int MMtoTicks = 10;

    // finals

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        motorFrontLeft = hardwareMap.dcMotor.get("UL");
        motorBackLeft = hardwareMap.dcMotor.get("DL");
        motorFrontRight = hardwareMap.dcMotor.get("UR");
        motorBackRight = hardwareMap.dcMotor.get("DR");

        elevatorRight = hardwareMap.dcMotor.get("ER");
        elevatorLeft = hardwareMap.dcMotor.get("EL");

        stickHolder = hardwareMap.servo.get("StickHolder");

        PUMPR = hardwareMap.crservo.get("pr");
        PUMPL = hardwareMap.crservo.get("pl");

        turnPump = hardwareMap.servo.get("rotatePump");
        PumpDirectionChangerRight = hardwareMap.servo.get("sideRight");
        PumpDirectionChangerLeft = hardwareMap.servo.get("sideLeft");

        sleeveSensor = hardwareMap.colorSensor.get("colorSensorLeft");

        // set direction of needed motors to reverse
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set encoders to 0
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorFrontLeft.setTargetPosition(0);


        // set encoders
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        turnPump.setPosition(1);
        PumpDirectionChangerLeft.setPosition(1);
        PumpDirectionChangerRight.setPosition(0);

        waitForStart();

        setElevatorHight(200);
        DriveHorisontaly(-600, 0.2);
        WaitForPosition();

        ScanForColor();

        telemetry.addData("Color", sleeveColor);
        telemetry.update();

//        setElevatorHight(2150);
//        DriveHorisontaly(-450,0.8);
//        DriveVerticaly(400,0.6);
//
//        ReleaseCone();
//
//        DriveVerticaly(-400,0.6);
//        DriveHorisontaly(400,0.75);

        if (sleeveColor == "blue") DriveVerticaly(1300, 0.3);
        if (sleeveColor == "red") DriveVerticaly(-1300, 0.3);
        WaitForPosition();
        sleep(1000);
    }
    //driveByEncoder(0.5, 0, 100, 0);

    public void reset() {
        motorBackRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorFrontLeft.setTargetPosition(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
    }

    public void setElevatorHight(int ticks) {
        elevatorRight.setTargetPosition(ticks);
        elevatorLeft.setTargetPosition(ticks);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
        elevatorLeft.setPower(1);
    }

    public void WaitForElevatorPosition() {
        while (!(elevatorRight.getCurrentPosition() < elevatorRight.getTargetPosition() + 5 &&
                elevatorRight.getCurrentPosition() > elevatorRight.getTargetPosition() - 5 &&
                elevatorLeft.getCurrentPosition() < elevatorLeft.getTargetPosition() + 5 &&
                elevatorLeft.getCurrentPosition() > elevatorLeft.getTargetPosition() - 5)) ;
    }

    //drives
    public void DriveVerticaly(int ticks, double power) {
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticks);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticks);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
    }

    public void DriveHorisontaly(int ticks, double power) {

        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() - ticks);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() - ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticks);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
    }

    public void Turn(int ticks) {
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() - ticks);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() - ticks);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
    }

    public void goToJunction(int x, int y) {
        if (firstJunction) {
            goToTheFirstJunction(x, y);
            firstJunction = false;
        } else {
            DriveVerticaly(-x * tiles, 1);
            WaitForPosition();
            DriveHorisontaly((y - 2) * tiles, 1);
        }
        Turn(650);
        WaitForPosition();
        setElevatorHight(junctionHight[x][y]);
        FindJunctionOrConeStack(0.97);
        // ReleaseCone();

        // setElevatorHight(ConeStackHight[coneInStack]);
        // DriveVerticaly(-400,1);
        // WaitForPosition();
        // Turn(1755);
        // if (coneInStack != 0){
        //     DriveHorisontaly((2-y)*tiles,1);
        //     DriveVerticaly((x-1)*tiles,1);
        //     FindTape(0.6);
        //     FindJunctionOrConeStack(0.6);

        //     CollectCone(0.8);
        //     coneInStack--;
        // } else{

        // }

    }

    public void ScanForColor() {
        if (sleeveSensor.red() > sleeveSensor.green() && sleeveSensor.red() > sleeveSensor.blue()) {
            sleeveColor = "red";
        }
        if (sleeveSensor.blue() > sleeveSensor.green() && sleeveSensor.blue() > sleeveSensor.red()) {
            sleeveColor = "blue";
        }
        if (sleeveSensor.green() > sleeveSensor.red() && sleeveSensor.green() > sleeveSensor.blue()) {
            sleeveColor = "green";
        }
        ScanForColor();
    }

    public void driveByEncoder(double power, double angleDegrees, int CM, int angleAheadDegrees) {
        // 9.6 CM Koter
        // 9.6 * PI = Hekef
        // Hekef = 30.16
        // encoder for one round = 537.6
        // (30.16 / 537.6) * encoder = CM
        // 0.0561 * encoder = CM
        // encoder = CM / 0.0561
        double encoder = CM / 0.0561;


        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runTime.reset();

        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getCurrentAngleFromIMU()) * 0.01;
//         Denominator is the largest motor power (absolute value) or 1
//         This ensures all the powers maintain the same ratio, but only when
//         at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double frontRightEncoder = encoder;
        double backRightEncoder = encoder;

        boolean motorFrontRightEncoderIsNotFinished = Math.abs(motorFrontRight.getCurrentPosition()) < frontRightEncoder;
//        boolean motorBackRightEncoderIsNotFinished = Math.abs(motorBackRight.getCurrentPosition()) < backRightEncoder;


//        while (motorFrontRightEncoderIsNotFinished || motorBackRightEncoderIsNotFinished) {
        while (motorFrontRightEncoderIsNotFinished) {

            drive(power, angleDegrees, angleAheadDegrees);
            motorFrontRightEncoderIsNotFinished = Math.abs(motorFrontRight.getCurrentPosition()) < frontRightEncoder;
//            rx = -(angleAheadDegrees - getCurrentAngleFromIMU()) * 0.01;
//            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////
////            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
////                stopRobot();
////            }
//            frontLeftPower = (y + x + rx) / denominator;
//            backLeftPower = (y - x + rx) / denominator;
//            frontRightPower = (y - x - rx) / denominator;
//            backRightPower = f(y + x - rx) / denominator;
//
//            motorFrontLeft.setPower(frontLeftPower);
//            motorBackLeft.setPower(backLeftPower);
//            motorFrontRight.setPower(frontRightPower);
//            motorBackRight.setPower(backRightPower);

        }
        stopRobot();
    }

    private void stopRobot() {
//        motorFrontLeft.setPower(0);
//        motorBackLeft.setPower(0);
//        motorFrontRight.setPower(0);
//        motorBackRight.setPower(0);

        stopOne(motorFrontLeft);
        stopOne(motorBackLeft);
        stopOne(motorFrontRight);
        stopOne(motorBackRight);

    }


    private void stopOne(DcMotor motor) {
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                motor.setPower(0);
            }
        });
    }

    public void goToTheFirstJunction(int x, int y) {


        telemetry.addData("Scaning...", "");
        telemetry.update();


        DriveHorisontaly(-tiles * (y - 1), 1);
        WaitForPosition();

        telemetry.addData("DH SECSSESFULL", "");
        telemetry.update();

        DriveVerticaly(tiles * (x - 1), 1);
        WaitForPosition();

        telemetry.addData("DH SECSSESFULL", "");
        telemetry.update();
    }

    private void CollectCone(double power) {
        PUMPL.setPower(-1);
        PUMPR.setPower(1);

        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);


        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopRobot();
    }

    private void ReleaseCone() {

        PUMPL.setPower(1);
        PUMPR.setPower(-1);

        sleep(1500);

        PUMPL.setPower(0);
        PUMPR.setPower(0);
    }

    private void FindJunctionOrConeStack(double power) {
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);


        stopRobot();
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        WaitForPosition();
    }

    public void WaitForPosition() {
        while (!((motorBackRight.getCurrentPosition() < motorBackRight.getTargetPosition() + 5 &&
                motorBackRight.getCurrentPosition() > motorBackRight.getTargetPosition() - 5) ||
                (motorBackLeft.getCurrentPosition() < motorBackLeft.getTargetPosition() + 5 &&
                        motorBackLeft.getCurrentPosition() > motorBackLeft.getTargetPosition() - 5) ||
                (motorFrontRight.getCurrentPosition() < motorFrontRight.getTargetPosition() + 5 &&
                        motorFrontRight.getCurrentPosition() > motorFrontRight.getTargetPosition() - 5) ||
                (motorFrontLeft.getCurrentPosition() < motorFrontLeft.getTargetPosition() + 5 &&
                        motorFrontLeft.getCurrentPosition() > motorFrontLeft.getTargetPosition() - 5)))
            ;
        stopRobot();
    }

    private void FindTape(double power) {
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);

        while (!(tapeSensor.red() > tapeSensor.blue() && tapeSensor.red() > tapeSensor.green())) ;

        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getCurrentAngleFromIMU() {


        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {

            deltaAngle += 360;

        } else if (deltaAngle > 180) {

            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return -globalAngle;
    }

    public void drive(double power, double angle, double rotation) {

        double y = -(power * Math.sin(angle));
        double x = -(power * Math.cos(angle));
        double rx = -rotation;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    public void DriveToJunction(int x, int y) {
        DriveVerticaly(-x * tiles, 0.75);
        WaitForPosition();
        DriveHorisontaly((y - 1) * tiles, 0.75);
        WaitForPosition();

        Turn(-1200);
        setElevatorHight(junctionHight[x][y]);
        WaitForPosition();
        DriveVerticaly(200, 0.75);
        WaitForPosition();

        PUMPR.setPower(1);
        PUMPL.setPower(-1);
        sleep(1500);
        PUMPR.setPower(0);
        PUMPL.setPower(0);

        DriveVerticaly(-200, 0.75);
        setElevatorHight(0);
        WaitForPosition();
        Turn(1200);
        WaitForPosition();

        DriveHorisontaly((y - 1) * -tiles, 0.75);
        WaitForPosition();
        DriveVerticaly(x * tiles, 0.75);
    }

}

