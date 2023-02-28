package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "SingleController", group = "BTJ")
public class Controller extends LinearOpMode {

    //Controller ***************************************************************

    //Vibration
    Gamepad.RumbleEffect EndGameVibration;
    Gamepad.RumbleEffect lastTenSecondsVibration;
    Gamepad.RumbleEffect readyToReleaseVibration;
    Gamepad.RumbleEffect closeToJunctionVibration;

    //Controller ***************************************************************

    //Drive variables***********************************************************

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double correction;
    double power;
    double driverAngle;
    double rotation;
    double targetAngle;

    //Drive variables***********************************************************

    //Mechanics***************************************************************

    //Wheels
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    //Elevators
    DcMotor elevatorRight;
    DcMotor elevatorLeft;
    //Pumping system
    CRServo PUMPR;
    CRServo PUMPL;
    Servo turnPump;
    Servo PumpDirectionChangerRight;
    Servo PumpDirectionChangerLeft;
    //Sensors
    TouchSensor stopP;

    DistanceSensor Ilay;

    //Mechanics***************************************************************

    //Program variables*********************************************************

    // IsPressed
    boolean xIsPressed = false;
    boolean stopPIsPressed = false;
    boolean elevatorIsUp = false;
    boolean BRIsPressed = false;
    boolean BLIsPressed = false;
    boolean aIsPressed = false;
    boolean bIsPressed = false;
    boolean yIsPressed = false;
    boolean rightTriggerWasPressed = false;
    boolean leftTriggerWasPressed = false;

    //algorithmic

    int minHightForJunctionIdentification = 700;
    int minHightForConeIdentification = 100;
    int maxElevatorHight = 3000;
    int minElevatorHight = 1;
    int targetHight = 0;
    final int[] junctionsLevels = new int[]{0, 1350, 2150, 2900};

    //Program variables*********************************************************

    @Override
    public void runOpMode() throws InterruptedException {

        //Configure*************************************************************

        //Find on hardware
        motorFrontLeft = hardwareMap.dcMotor.get("UL");
        motorBackLeft = hardwareMap.dcMotor.get("DL");
        motorFrontRight = hardwareMap.dcMotor.get("UR");
        motorBackRight = hardwareMap.dcMotor.get("DR");

        elevatorRight = hardwareMap.dcMotor.get("ER");
        elevatorLeft = hardwareMap.dcMotor.get("EL");

        turnPump = hardwareMap.servo.get("rotatePump");
        PumpDirectionChangerRight = hardwareMap.servo.get("sideRight");
        PumpDirectionChangerLeft = hardwareMap.servo.get("sideLeft");

        PUMPR = hardwareMap.crservo.get("pr");
        PUMPL = hardwareMap.crservo.get("pl");

        stopP = hardwareMap.touchSensor.get("stopPump");

        Ilay = hardwareMap.get(DistanceSensor.class, "Ilay");

        //Settings
        Ilay.resetDeviceConfigurationForOpMode();

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorRight.setTargetPosition(0);
        elevatorLeft.setTargetPosition(0);

        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Configure*************************************************************

        //Controller****************************************************************

        EndGameVibration = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 250)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 250)
                .build();

        lastTenSecondsVibration = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 2000)
                .addStep(0.0, 0.0, 2000)
                .addStep(0.7, 0.7, 2000)
                .addStep(0.0, 0.0, 2000)
                .addStep(1.0, 1.0, 2000)
                .build();

        readyToReleaseVibration = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 250)
                .build();

        //Controller************************************************************

        //INIT movements********************************************************

        turnPump.setPosition(0);
        PumpDirectionChangerLeft.setPosition(1);
        PumpDirectionChangerRight.setPosition(0);

        //INIT movements********************************************************

        waitForStart();//*******************************************************

        //GameTimer*************************************************************

        new Thread(new Runnable() {
            @Override
            public void run() {
                sleep(90000);
                gamepad1.runRumbleEffect(EndGameVibration);
                sleep(20000);
                gamepad1.runRumbleEffect(lastTenSecondsVibration);
            }
        }).start();

        //GameTimer*************************************************************

        while (opModeIsActive()) {

            drive(power, targetAngle, rotation);

            // if(Ilay.getDistance(DistanceUnit.CM) < 18 && elevatorRight.getCurrentPosition() > 200 && !distanceIsClose){
            //     distanceIsClose = true;
            //     motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //     motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //     motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //     motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //     motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition());
            //     motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition());
            //     motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition());
            //     motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition());

            //     motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //     motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //     motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //     motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //     motorFrontLeft.setPower(1);
            //     motorBackLeft.setPower(1);
            //     motorFrontRight.setPower(1);
            //     motorBackRight.setPower(1);

            //     PUMPR.setPower(1);
            //     PUMPL.setPower(-1);
            //     sleep(500);
            // }
            // if(Ilay.getDistance(DistanceUnit.CM) > 18){
            //     distanceIsClose = false;
            // }

            // if(elevatorRight.getCurrentPosition()>=900 && !elevatorIsUp)
            // {
            //     elevatorIsUp = true;
            // }
            // else if(elevatorRight.getCurrentPosition()<900 && elevatorIsUp)
            // {
            //     elevatorIsUp = false;
            // }

            if (gamepad1.a && !aIsPressed) {
                aIsPressed = true;
                if (PUMPR.getPower() != 1) {
                    PUMPR.setPower(1);
                    PUMPL.setPower(-1);
                } else {
                    PUMPR.setPower(0);
                    PUMPL.setPower(0);
                }
            } else if (!gamepad1.a && aIsPressed) {
                aIsPressed = false;
            }

            if (gamepad1.b && !bIsPressed) {
                bIsPressed = true;
                if (PUMPR.getPower() != -1) {
                    PUMPR.setPower(-1);
                    PUMPL.setPower(1);
                } else {
                    PUMPR.setPower(0);
                    PUMPL.setPower(0);
                }
            } else if (!gamepad1.b && bIsPressed) {
                bIsPressed = false;
            }

            if (gamepad1.y && !yIsPressed) {
                yIsPressed = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        if (PumpDirectionChangerLeft.getPosition() == 0) {
                            turnPump.setPosition(0);
                            sleep(400);
                            PumpDirectionChangerLeft.setPosition(1);
                            PumpDirectionChangerRight.setPosition(0);
                        } else if (PumpDirectionChangerRight.getPosition() == 0) {
                            turnPump.setPosition(0.85);
                            sleep(400);
                            PumpDirectionChangerLeft.setPosition(0);
                            PumpDirectionChangerRight.setPosition(1);
                        }
                    }
                }).start();
            } else if (!gamepad1.y) {
                yIsPressed = false;
            }


            if (gamepad1.x && !xIsPressed) {
                xIsPressed = true;

                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        if (PumpDirectionChangerLeft.getPosition() == 0) {
                            targetHight = 0;
                            setElevatorHeight(0);
                            turnPump.setPosition(0);
                            sleep(350);
                            PumpDirectionChangerLeft.setPosition(1);
                            PumpDirectionChangerRight.setPosition(0);
                        } else if (PumpDirectionChangerRight.getPosition() == 0) {
                            targetHight = 3;
                            setElevatorHeight(3);
                            turnPump.setPosition(0.85);
                            sleep(350);
                            PumpDirectionChangerLeft.setPosition(0);
                            PumpDirectionChangerRight.setPosition(1);
                        }
                    }
                }).start();
            }

            if (!gamepad1.x && xIsPressed) {
                xIsPressed = false;
            }


            if (gamepad1.dpad_left) {
                turnPump.setPosition(0.85);
            }
            if (gamepad1.dpad_right) {
                turnPump.setPosition(0.17);
            }
            if (gamepad1.dpad_up) {
                PumpDirectionChangerLeft.setPosition(1);
                PumpDirectionChangerRight.setPosition(0);
            }
            if (gamepad1.dpad_down) {
                PumpDirectionChangerLeft.setPosition(0);
                PumpDirectionChangerRight.setPosition(1);
            }

            if (Ilay.getDistance(DistanceUnit.CM) < 15 && Ilay.getDistance(DistanceUnit.CM) > 5 && elevatorLeft.getCurrentPosition() > minHightForJunctionIdentification) {
                gamepad1.runRumbleEffect(readyToReleaseVibration);
            } else if (Ilay.getDistance(DistanceUnit.CM) < 20 && Ilay.getDistance(DistanceUnit.CM) > 15 && elevatorLeft.getCurrentPosition() > minHightForJunctionIdentification) {
                closeToJunctionVibration = new Gamepad.RumbleEffect.Builder()
                        .addStep((20 - Ilay.getDistance(DistanceUnit.CM)) / 5, 0.0, 100)
                        .build();
                gamepad1.runRumbleEffect(closeToJunctionVibration);
            } else if (Ilay.getDistance(DistanceUnit.CM) < 5 && elevatorLeft.getCurrentPosition() > minHightForJunctionIdentification) {
                closeToJunctionVibration = new Gamepad.RumbleEffect.Builder()
                        .addStep(0.0, Ilay.getDistance(DistanceUnit.CM) / 5, 100)
                        .build();
                gamepad1.runRumbleEffect(closeToJunctionVibration);
            }

            if (stopP.isPressed() && elevatorLeft.getCurrentPosition() < minHightForConeIdentification && !stopPIsPressed) {
                stopPIsPressed = true;
                PUMPR.setPower(0);
                PUMPL.setPower(0);
                if (elevatorLeft.getTargetPosition() < minHightForConeIdentification) {
                    elevatorRight.setTargetPosition(150);
                    elevatorLeft.setTargetPosition(150);
                }
                elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorRight.setPower(1);
                elevatorLeft.setPower(1);
            } else if (stopPIsPressed && !stopP.isPressed()) {
                stopPIsPressed = false;
            }

            //these two ifs are responsible for deciding the height of the elevator
            // (dpad_up --> elevator up , dpad_down --> elevator reset to 0)

            if (gamepad1.right_bumper && !BRIsPressed) {
                BRIsPressed = true;
//                if(elevatorRight.getCurrentPosition()<junctionsLevels[1])
//                    setElevatorHeight(1);
//                else if(elevatorRight.getCurrentPosition()<junctionsLevels[2])
//                    setElevatorHeight(2);
//                else setElevatorHeight(3);
                targetHight++;
                if (targetHight > 3) targetHight = 3;
                setElevatorHeight(targetHight);
            } else if (!gamepad1.right_bumper && BRIsPressed) {
                BRIsPressed = false;
            }

            if (gamepad1.left_bumper && !BLIsPressed) {
                BLIsPressed = true;
//                if(elevatorRight.getCurrentPosition()>junctionsLevels[3])
//                    setElevatorHeight(3);
//                else if (elevatorRight.getCurrentPosition()>junctionsLevels[2])
//                    setElevatorHeight(2);
//                else if (elevatorRight.getCurrentPosition()>junctionsLevels[1])
//                    setElevatorHeight(1);
//                else setElevatorHeight(0);
                targetHight--;
                if (targetHight <= 0) targetHight = 0;
                setElevatorHeight(targetHight);
            } else if (!gamepad1.left_bumper && BLIsPressed) {
                BLIsPressed = false;
            }


            //to do max and min elevating
            if (gamepad1.right_trigger > 0 && elevatorRight.getCurrentPosition() < maxElevatorHight) {
                rightTriggerWasPressed = true;
                elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorRight.setPower(gamepad1.right_trigger);
                elevatorLeft.setPower(gamepad1.right_trigger);
            } else if (rightTriggerWasPressed) {
                rightTriggerWasPressed = false;
                elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorRight.setTargetPosition(elevatorRight.getCurrentPosition());
                elevatorLeft.setTargetPosition(elevatorLeft.getCurrentPosition());
                elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorRight.setPower(1);
                elevatorLeft.setPower(1);

                if (elevatorRight.getCurrentPosition() > junctionsLevels[2] + 10)
                    targetHight = 3;
                else if (elevatorRight.getCurrentPosition() > junctionsLevels[1] + 10)
                    targetHight = 2;
                else if (elevatorRight.getCurrentPosition() > junctionsLevels[0] + 10)
                    targetHight = 1;
                else targetHight = 0;
            }

            if (gamepad1.left_trigger > 0 && elevatorRight.getCurrentPosition() > minElevatorHight) {
                leftTriggerWasPressed = true;
                elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorRight.setPower(-gamepad1.left_trigger);
                elevatorLeft.setPower(-gamepad1.left_trigger);
            } else if (leftTriggerWasPressed) {
                leftTriggerWasPressed = false;
                elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorRight.setTargetPosition(elevatorRight.getCurrentPosition());
                elevatorLeft.setTargetPosition(elevatorLeft.getCurrentPosition());
                elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorRight.setPower(1);
                elevatorLeft.setPower(1);

                if (elevatorRight.getCurrentPosition() > junctionsLevels[2] + 10)
                    targetHight = 3;
                else if (elevatorRight.getCurrentPosition() > junctionsLevels[1] + 10)
                    targetHight = 2;
                else if (elevatorRight.getCurrentPosition() > junctionsLevels[0] + 10)
                    targetHight = 1;
                else targetHight = 0;
            }

            //Drive actions*********************************************************

            power = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));

            driverAngle = Math.atan(-gamepad1.left_stick_y / gamepad1.left_stick_x);

            rotation = gamepad1.right_stick_x;

            if (Double.isNaN(driverAngle))
                driverAngle = 0;

            if (gamepad1.left_stick_x < 0)
                driverAngle += Math.PI;

            targetAngle = driverAngle; //- (-Math.toRadians(getAngle()));

            //Drive actions*****************************************************
        }
    }

    //Drive functions***********************************************************

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

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
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

    //Drive functions***********************************************************

    public void setElevatorHeight(int targetHight) {
        // ths function sets the elevator heights to a specific hight by the encoder
        // and the junction levels
        elevatorRight.setTargetPosition(junctionsLevels[targetHight]);
        elevatorLeft.setTargetPosition(junctionsLevels[targetHight]);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
        elevatorLeft.setPower(1);
    }
}