package org.firstinspires.ftc.teamcode.olf;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous By Encoder Left")
// @Disabled

public class AutonomusByDistanceLeft extends LinearOpMode {
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    boolean EndTime = false;

    Servo turnPump;
    Servo PumpDirectionChangerRight;
    Servo PumpDirectionChangerLeft;

    CRServo PUMPR, PUMPL;

    int tiles = 1200;
    int stackHightIndex = 0;

    // final int[] ConeStackHight = new int[]{0,100,200,300,400,500};
    int coneInStack = 5;

    DcMotor elevatorRight;
    DcMotor elevatorLeft;

    // int[] coneStackHights = new int[]{155,120,75,35,0};
    int[] coneStackHights = new int[]{410,308,205,100,0};
    // int[] coneStackHights = new int[]{-35,-65,-105,-145};

    int[][] junctionHight = new int[][]{
            {0,1350,0,1350,0},
            {1350,2150,2900,2150,1350},
            {0,2900,0,2900,0},
            {1350,2150,2900,2150,1350},
            {0,1350,0,1350,0}};
    // int[] coneStackHight = new int[] {100,200,300,400};
    int currentConeInStack = 3;

    ColorSensor sleeveSensor;
    ColorSensor tapeSensor;

    DistanceSensor distanceRight;
    DistanceSensor distanceLeft;
    DistanceSensor Ilay;

    TouchSensor stopP;

    boolean firstJunction = true;

    public ElapsedTime runTime = new ElapsedTime();

    String sleeveColor = "none";

    int Red = 1300;
    int Green = 3500;
    int Blue = 2500;
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

        distanceRight = hardwareMap.get(DistanceSensor.class, "DistanceRight");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "DistanceLeft");
        Ilay = hardwareMap.get(DistanceSensor.class, "Ilay");

        stopP = hardwareMap.touchSensor.get("stopPump");

        PUMPR = hardwareMap.crservo.get("pr");
        PUMPL = hardwareMap.crservo.get("pl");

        sleeveSensor = hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        turnPump = hardwareMap.servo.get("rotatePump");
        PumpDirectionChangerRight = hardwareMap.servo.get("sideLeft");
        PumpDirectionChangerLeft = hardwareMap.servo.get("sideRight");

        // set direction of needed motors to reverse
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set encoders to 0
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorFrontLeft.setTargetPosition(0);

        runTime.reset();


        // set encoders
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PUMPR.setPower(-1);
        PUMPL.setPower(1);
        turnPump.setPosition(0);
        PumpDirectionChangerLeft.setPosition(0);
        PumpDirectionChangerRight.setPosition(1);
        PUMPR.setPower(0);
        PUMPL.setPower(0);

        waitForStart();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()){
                    telemetry.addData("FR",motorFrontRight.getCurrentPosition());
                    telemetry.addData("BR",motorBackRight.getCurrentPosition());
                    telemetry.update();}
            }
        }).start();

        //**********************************************************************************************************************************************************/\

        setElevatorHight(coneStackHights[stackHightIndex]);
        for(int i = 0; i < 4; i++){
            stackHightIndex++;
            driveByEncoder(1,0,45,0);
            sleep(200);
            driveToWall(0.3,0,0);
            PUMPR.setPower(1);
            PUMPL.setPower(-1);
            sleep(200);
            PUMPR.setPower(0);
            PUMPL.setPower(0);
            sleep(200);
            setElevatorHight(2900);
            sleep(200);
            new Thread(new Runnable(){
                @Override
                public void run() {
                    turnPump.setPosition(0.85);
                    sleep(300);
                    PumpDirectionChangerLeft.setPosition(1);
                    PumpDirectionChangerRight.setPosition(0);
                }
            }).start();
            driveByEncoder(-1,0,57,0);
            sleep(500);
            driveByEncoder(0.5,90,37,0);
            sleep(500);
            driveByEncoder(-0.5,0,10,0);
            sleep(200);
            driveToJunction(-0.15,0,0);
            PUMPR.setPower(1);
            PUMPL.setPower(-1);
            sleep(500);
            new Thread(new Runnable(){
                @Override
                public void run() {
                    sleep(400);
                    setElevatorHight(coneStackHights[stackHightIndex]);
                    turnPump.setPosition(0);
                    sleep(300);
                    PumpDirectionChangerLeft.setPosition(0);
                    PumpDirectionChangerRight.setPosition(1);
                }
            }).start();
            driveByEncoder(0.3,0,13,0);
            sleep(500);
            PUMPR.setPower(0);
            PUMPL.setPower(0);
            sleep(200);
            driveByEncoder(0.5,-90,37,0);
            sleep(500);
        }
        stackHightIndex++;
        driveByEncoder(1,0,45,0);
        sleep(200);
        driveToWall(0.3,0,0);
        PUMPR.setPower(1);
        PUMPL.setPower(-1);
        sleep(200);
        PUMPR.setPower(0);
        PUMPL.setPower(0);
        sleep(1000);
        setElevatorHight(2900);
        sleep(200);
        new Thread(new Runnable(){
            @Override
            public void run() {
                turnPump.setPosition(0.85);
                sleep(300);
                PumpDirectionChangerLeft.setPosition(1);
                PumpDirectionChangerRight.setPosition(0);
            }
        }).start();
        driveByEncoder(-1,0,57,0);
        sleep(500);
        driveByEncoder(0.5,90,37,0);
        sleep(500);
        driveByEncoder(-0.5,0,10,0);
        sleep(200);
        driveToJunction(-0.15,0,0);
        PUMPR.setPower(1);
        PUMPL.setPower(-1);
        sleep(500);
        new Thread(new Runnable(){
            @Override
            public void run() {
                sleep(400);
                setElevatorHight(coneStackHights[stackHightIndex]);
                turnPump.setPosition(0);
                sleep(300);
                PumpDirectionChangerLeft.setPosition(0);
                PumpDirectionChangerRight.setPosition(1);
            }
        }).start();
        driveByEncoder(0.3,0,13,0);
        sleep(500);
        PUMPR.setPower(0);
        PUMPL.setPower(0);
        driveByEncoder(0.5,-90,37,0);
        stopRobot();
        sleep(100000);
        //***********************************************************************************************************************************************//
        driveByEncoder(0.5,-90,3,0);
    }

    public  void reset(){
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
    public void setElevatorHight(int ticks){
        elevatorRight.setTargetPosition(ticks);
        elevatorLeft.setTargetPosition(ticks);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
        elevatorLeft.setPower(1);
    }

    //drives

    public void ScanForColor(){
        if(sleeveSensor.red() > sleeveSensor.green() && sleeveSensor.red() > sleeveSensor.blue()) {
            sleeveColor = "red";
            return;
        }if(sleeveSensor.blue() > sleeveSensor.green() && sleeveSensor.blue() > sleeveSensor.red()){
            sleeveColor = "blue";
            return;
        }if(sleeveSensor.green() > sleeveSensor.red() && sleeveSensor.green() > sleeveSensor.blue()){
            sleeveColor = "green";
            return;
        }
        ScanForColor();
    }

    private void stopRobot() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;


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

        motorFrontLeft.setPower(-frontLeftPower);
        motorBackLeft.setPower(-backLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackRight.setPower(-backRightPower);
    }

    public void driveByEncoder(double power, double angleDegrees, double CM, int angleAheadDegrees) {
        // 9.6 CM Koter
        // 9.6 * PI = Hekef
        // Hekef = 30.16
        // encoder for one round = 537.6
        // (30.16 / 537.6) * encoder = CM
        // 0.0561 * encoder = CM
        // encoder = CM / 0.0561
        double encoder = CM / 0.0561;

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();
        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getAngle()) * 0.01;
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


        while (Math.abs(motorFrontRight.getCurrentPosition()) < frontRightEncoder || Math.abs(motorBackRight.getCurrentPosition()) < backRightEncoder) {
            rx = -(angleAheadDegrees - getAngle()) * 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public void turnToZero(){
        while(getAngle() < 75){
            drive(0,0,0.7);
        }
        stopRobot();
    }

    public void turnToJunction(){
        while(getAngle() > 45){
            drive(0,0,-0.7);
        }
        stopRobot();
    }

    public void turnToFirstJunction(){
        while(getAngle() < 45){
            drive(0,0,0.7);
        }
        stopRobot();
    }

    public void driveToWall(double power, double angleDegrees, int angleAheadDegrees) {
        // 9.6 CM Koter
        // 9.6 * PI = Hekef
        // Hekef = 30.16
        // encoder for one round = 537.6
        // (30.16 / 537.6) * encoder = CM
        // 0.0561 * encoder = CM
        // encoder = CM / 0.0561
        // double encoder = CM / 0.0561;

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();
        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getAngle()) * 0.01;
//         Denominator is the largest motor power (absolute value) or 1
//         This ensures all the powers maintain the same ratio, but only when
//         at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // double frontRightEncoder = encoder;
        // double backRightEncoder = encoder;


        while (verifyDistance()) {
            rx = -(angleAheadDegrees - getAngle()) * 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public boolean verifyDistance(){
        int counter = 0;
        boolean current = false;
        while(counter < 3){
            current = distanceRight.getDistance(DistanceUnit.CM) > 3 || distanceLeft.getDistance(DistanceUnit.CM) > 3;
            if(current == distanceRight.getDistance(DistanceUnit.CM) > 3 || distanceLeft.getDistance(DistanceUnit.CM) > 3){
                counter++;
            }
            else{
                counter = 0;
            }
        }
        return current;
    }
    public void driveToJunction(double power, double angleDegrees, int angleAheadDegrees) {
        // 9.6 CM Koter
        // 9.6 * PI = Hekef
        // Hekef = 30.16
        // encoder for one round = 537.6
        // (30.16 / 537.6) * encoder = CM
        // 0.0561 * encoder = CM
        // encoder = CM / 0.0561
        // double encoder = CM / 0.0561;

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();
        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getAngle()) * 0.01;
//         Denominator is the largest motor power (absolute value) or 1
//         This ensures all the powers maintain the same ratio, but only when
//         at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // double frontRightEncoder = encoder;
        // double backRightEncoder = encoder;


        while (verifyIlay()) {
            rx = -(angleAheadDegrees - getAngle()) * 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public boolean verifyIlay(){
        int counter = 0;
        boolean current = false;
        while(counter < 3){
            current = Ilay.getDistance(DistanceUnit.CM) > 20;
            if(current == Ilay.getDistance(DistanceUnit.CM) > 20){
                counter++;
            }
            else{
                counter = 0;
            }
        }
        return current;
    }
}

