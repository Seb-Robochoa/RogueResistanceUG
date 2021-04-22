package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingRunnable;
import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingSupplier;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.ArrayList;


import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.Timer;
//import gayness
@Autonomous
public class AutoMain extends LinearOpMode {
    private static double forwardAngle;
    private final int[] RIGHT = {1, 2, 2, 1},
            LEFT = {2, 1, 1, 2},
            FORWARD = {1, 1, 1, 1},
            BACKWARD = {2, 2, 2, 2},
            FOWARDRIGHT = {1, 0, 0, 1},
            FORWARDLEFT = {0, 1, 1, 0},
            BACKWARDRIGHT = {0, 2, 2, 0},
            BACKWARDLEFT = {2, 0, 0, 2},
            TURNRIGHT = {1, 2, 1, 2},
            TURNLEFT = {2, 1, 2, 1};

    private DcMotorEx leftFront, rightFront, leftBack, rightBack, arm, transfer, intake, shooter;
    private Servo claw, flicker, holder;
    private DcMotorEx[] motors;

    private final int WHEEL_RADIUS = 4;
    private final double GEAR_RATIO = (double) 5 / 6;
    private final double TICKS_PER_REVOLUTION = 537.6;

    private final double TPI = TICKS_PER_REVOLUTION / (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS);
    private int scenario;
    private ElapsedTime runtime;
    OpenCvInternalCamera phoneCam;
    BNO055IMU imu;

    AutoMain.UltimateGoalDeterminationPipeline pipeline;
    private int distance;
    private int x;
    private int y;
    Orientation angles;
    private String currentDirection;
    Orientation ang;
    double forwardAngles;
    double turnKp;
    private double initialAngle;
    private ArrayList<Double[]> storage;


    //PID stuff
    //private PID forwardPID = new PID(.022, 0.0003, 0.0022);
    // private PID strafePID = new PID(.02, .0003, 0.002);
    //.0033
    private PID forwardPID = new PID(.022, 0, 0.0033);
    private PID strafePID = new PID(.024, 0, 0.0033);

    //private PID forwardPID = new PID(.0152, 0, 0);
    //private PID strafePID = new PID(.015, 0, 0);

    @Override
    //Main method which runs when you click AutoMain on the phone
    public void runOpMode() throws InterruptedException { //Load Zone B
        //Intialize all motors and encoders
        initialize();

        //Moves the bot forward
        /*(10);snapBot();pause(500);
        turn(-10);snapBot();pause(500);
        turn(15);snapBot();pause(500);
        turn(-15);snapBot();pause(500);
        turn(90);snapBot();pause(500);
        turn(-90);snapBot();pause(500);
        turn(45);snapBot();pause(500);
        turn(-45);snapBot();pause(500);
        turn(30);snapBot();pause(500);
        turn(-30);snapBot();pause(500);
        turn(120);snapBot();pause(500);
        turn(-120);snapBot();pause(500);//test degrees close to 180, as 180 doesnt work with snap
        */

        //If there are only 0 rings detected by the robot
        shooter.setPower(-.76);
        setWobbler(scenario);


        //powerShot();
        snapBot();

        updateTelemetry();

        //strafeByWheelEncoders(0, 7, .6, "strafe left"); pause(750);


        //0 Void 1 Forward 2 Reverse


    }

    public void setWobbler(int scenario) throws InterruptedException {

        if (scenario == 0) //zone A
        {

            //moveBot(FORWARD, 56, .25, "straight"); //forward\

            moveByWheelEncoders(0, 80, .7, "straight");
            holder.setPosition(0);
            turn(-15);
            armTravel();
            turn(15);
            moveByWheelEncoders(0, 8, -.7, "straight");pause(500);snapBot();
            strafeByWheelEncoders(0, 10, .4, "strafe right");

            snapBot();

            yeetRing(-.64);

            strafeByWheelEncoders(0, 9, .6, "strafe right"); //22 of powershot
            // moveBot(RIGHT, 37, .85, "straferight");

            secondWobbler(42, 28, 40,"left");

            strafeByWheelEncoders(0, 25, 1, "strafe right");
            moveByWheelEncoders(0, 10, .7, "straight");
            snapBot();


        }
        if (scenario == 1) { //zone B//sdkjfghdfsugudrthv8suitjyukewhy7rehgjgytraysufewezfesiutsjgr67jyetym
            moveByWheelEncoders(0, 1, .7, "straight");
            strafeByWheelEncoders(0, 4, .4, "strafe left");pause(250); snapBot();
            moveByWheelEncoders(0, 70, .7, "straight"); pause(500); snapBot();
            strafeByWheelEncoders(0, 17, .4, "strafe right");
            snapBot();

            //armTravel();
            yeetRing(-.64);
            holder.setPosition(0);
            moveByWheelEncoders(0, 26, .7, "straight");
            turn(10);
            armTravel();
            turn(-10);
            shooter.setPower(-.71);


            moveByWheelEncoders(0,36,-.7,"straight");

            snapBot();

            strafeByWheelEncoders(0, 2, .5, "strafe left");
            loadMoreRings(false,-.75,false);

            strafeByWheelEncoders(0, 8, 1, "strafe right"); //22 of powershot
            snapBot();
            secondWobbler(24, 12, 70,"right");
            moveByWheelEncoders(0,20,-1,"straight");




            // moveByWheelEncoders(0,-6,.4,"straight");

            //strafeByWheelEncoders(0, 32.5, .4, "strafe right");

        }
        if (scenario == 4) { //zone C
            moveByWheelEncoders(0, 1, .7, "straight");pause(100);snapBot();
            strafeByWheelEncoders(0, 4, .4, "strafe left");pause(150);snapBot();
            moveByWheelEncoders(0, 70, .7, "straight");pause(150);snapBot();
            strafeByWheelEncoders(0, 14, .3, "strafe right");


            holder.setPosition(0);
            snapBot();
            //armTravel();
            yeetRing(-.64);

            strafeByWheelEncoders(0, 4, .5, "strafe right");

            moveByWheelEncoders(0, 13, -1, "straight");
            //moveByWheelEncoders(0, 2, .4, "straight");
            snapBot();

            moveByWheelEncoders(0, 2, 1, "straight");
            shooter.setPower(-.67);

            loadMoreRings(true, -.63,false);

            shooter.setPower(-.75);

            snapBot();
            loadMoreRings(false, -.72, true);

            strafeByWheelEncoders(0, 20, 1, "strafe left");
            snapBot();

            moveByWheelEncoders(0, 92, 1, "straight");

            armTravel();

            moveByWheelEncoders(0, 38, -1, "straight");
        }

    }


    public void snapBot() throws InterruptedException {//sdkfhgskhgdfsgklhsdfgljsfgjhfjgjhfgjhfgjhfgjhfgjkhfgjhfgjhfgjhfgjhfgjhfgjhfgjhfgjh
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        turn(Math.abs(initialAngle - angles.firstAngle) * (angles.firstAngle / Math.abs(angles.firstAngle)));
    }

    //Sets the robot angle back to zero
    public void backZero(int time) throws InterruptedException {
        ElapsedTime test = new ElapsedTime();
        //The robot tries to turn back to the original angle in 2.5 seconds
        while (!(((int) currentAngle() > -1) && ((int) currentAngle() < 1)) && test.milliseconds() <= 2250) {
            turn(currentAngle());

            if (((int) currentAngle() > -1) && ((int) currentAngle() < 1))
                break;
        }

    }

    //This will only be used for Zone C where the Robot can intake and shoot more rings
    public void loadMoreRings(boolean zoneC, double pow, boolean adjust) throws InterruptedException {


        intake(.6, zoneC);


        //moveBot(RIGHT, targetX-getX(), .75, "straferight");
        //updateX(targetX-getX());
        //int save = targetX-getX();

        snapBot();

        //moveByWheelEncoders(8,.6,1,"straight");

        //turn(5) ;
        if(!zoneC){
            yeetLessRing(pow);
        }
        else {
            if(adjust){
                moveByWheelEncoders(0,16,.7,"straight");
            }
            yeetRing(pow);
        }
        //turn(-5);

        // moveByWheelEncoders(0, 35, 1, "forward");


        //moveBot(LEFT, -save, .75, "strafeleft"); //right
        //updateX(save);
        //intake(.2);
        //moveBot(RIGHT, save, .75, "straferight");
        //yeetRing(-.73);


        //moveBot(1,2,2,1,4, .6, true); //strafe right
        //updateX(24);

        // holder.setPosition(0);
        // moveBot(1,1,1,1,-12, .6, true); // backward
        //updateY(48);
        // intake();
        //moveBot(1,2,2,1,-4, .6, true); //strafe left
        //updateX(16);
        // moveBot(1,1,1,1,12, .6, true); // forward
        //updateY(60);
        updateTelemetry();
    }

    //Gets the second Wobbler in the game
    public void secondWobbler(int forward, int targetX, int targetY, String strafeDir) throws InterruptedException {
        // position of the second wobbler is at 24, 5
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(-2500);

        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(180);
        pause(600);

        moveByWheelEncoders(180, forward, .4, "straight");
        pause(600);

        ElapsedTime wobbler = new ElapsedTime();

        while (wobbler.milliseconds() <= 2000) {
            heartbeat();
            if (arm.getCurrentPosition() > (arm.getTargetPosition() - 50)) {
                claw.setPosition(1);
                break;
            }
        }
        pause(250);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(2500);

        arm.setPower(1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.getCurrentPosition() < 2450) {
            heartbeat();
        }
        arm.setPower(0);

        snapBot();

        strafeByWheelEncoders(0, targetX, .7, "strafe "+strafeDir);
        moveByWheelEncoders(0, targetY, .7, "straight");

        armTravel();

    }


    //Not really important, just for testing purposes
    public void updateTelemetry() {
        telemetry.addData("X", getX());
        telemetry.addData("Y", getY());
        telemetry.update();
    }

    //The next for methods are getters and setter methods for ACCESS coordinates (X, Y)
    public void updateX(int pos) {
        x += pos;
    }

    public void updateY(int pos) {
        y += pos;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    //Initialize all the variables before the start of the match
    public void initialize() {
        //Even if I'm not using it, I have to map it because it is mapped on the bot.
        x = 0;
        y = 0;
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFrontDrive");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftRearDrive");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFrontDrive");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightRearDrive");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        flicker = hardwareMap.servo.get("flicker");
        holder = hardwareMap.servo.get("holder");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new AutoMain.UltimateGoalDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        claw.setPosition(1);
        flicker.setPosition(.7);
        storage = new ArrayList<Double[]>();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        runtime = new ElapsedTime();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initialAngle = angles.firstAngle;
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.openCameraDeviceAsync(() -> {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });

        while (!opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("TPI", TPI);
            telemetry.addData("TPR", TICKS_PER_REVOLUTION);
            telemetry.addData("GR", GEAR_RATIO);
            telemetry.addData("WR", WHEEL_RADIUS);


            //telemetry.addData(leftBack.getTargetPosition)
            updateTelemetry();
            telemetry.update();
            scenario = 1;
            if (pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.NONE) //zone A
            {
                scenario = 0;
            }
            if (pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.ONE) //zone A
            {
                scenario = 1;
            }
            if (pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.FOUR) //zone A
            {
                scenario = 4;
            }
            sleep(50);
        }
        motors = new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (motor == leftFront || motor == leftBack)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }
        waitForStart();
    }

    public void halt() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    /**
     * @param power
     * @param distance inches so you will have to convert to tics
     */

    //This method accepts 6 variables:
    // 4 of them are motors (leftFront, leftBack, rightFront, rightBack)
    //These variables will accept 3 integers: 0 Void, 1 Forward, 2 Reverse
    //It also accepts power and distance

    //withIntake will be used later (work in progress)

    //Move bot based on the direction with specific distance and power
    //0 = Void Wheel
    //1 = Set the motor to FORWARD
    //2 = Set the motor to REVERSE
    public void moveBot(int[] dir, int distance, double power, boolean withIntake) throws InterruptedException {
        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwward
        //turnBot(2, 2, 1, 1, -24, .60); //Backward
        //turnBot(2, 1, 2, 1, 30, .60); //Strafe right
        //turnBot(1, 2, 1, 2, 0, .6) /Strafe left
        int leftT = dir[0];
        int rightT = dir[1];
        int leftB = dir[2];
        int rightB = dir[3]; // MAKE CASE FOR 0
        if (leftT == 1) {
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        } else if (leftT == 2) {
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (leftB == 1) {
            leftBack.setDirection(DcMotor.Direction.FORWARD);
        } else if (leftB == 2) {
            leftBack.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightT == 2) {
            rightFront.setDirection(DcMotor.Direction.FORWARD);
        } else if (rightT == 1) {
            rightFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightB == 2) {
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        } else if (rightT == 1) {
            rightBack.setDirection(DcMotor.Direction.REVERSE);
        }

        /*if(withIntake) {
            while (shooterTime.milliseconds() <= 5000) {
                intake.setPower(.5);
                transfer.setPower(1);
                heartbeat();
            }
        }*/

        //Moves the robot
        int travel = (int) (distance * TPI);
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(travel);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //This is what checks if the motors are supposed to be still running.
        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            heartbeat();
        }
        //intake.setPower(0);
        // transfer.setPower(0);
    }

    public boolean motorsBusy(int ticks, double startingPosition) {
        return Math.abs(leftBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(leftFront.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightFront.getCurrentPosition() - startingPosition) < ticks;
    }


    public void moveByWheelEncoders(double targetHeading, double inches, double power, String movementType) throws InterruptedException {
        resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * TPI);

        while (motorsBusy(ticks, currentPosition)) {
            heartbeat();
            correction(-power, targetHeading, movementType, false, 1);
        }

        halt();
    }
    public void strafeByWheelEncoders(double targetHeading, double inches, double power, String movementType) throws InterruptedException {
        resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * TPI * 1.2);

        while (motorsBusy(ticks, currentPosition)) {
            heartbeat();
            correction(-power, targetHeading, movementType, false, 1);
        }

        halt();
    }


    public void lowestFactor() throws InterruptedException{
        double smallestAngle = Math.abs(storage.get(0)[4]);
        int j = 0;

        for(int i = 1; i < storage.size(); i++){
            if(smallestAngle > Math.abs(storage.get(i)[4])){
                smallestAngle = Math.abs(storage.get(i)[4]);
                j = i;
            }
        }

        double save1 = leftFront.getPower();
        double save2 = leftBack.getPower();
        double save3 = rightFront.getPower();
        double save4 = rightBack.getPower();


        leftFront.setPower(storage.get(j)[0]);
        leftBack.setPower(storage.get(j)[1]);
        rightFront.setPower(storage.get(j)[2]);
        rightBack.setPower(storage.get(j)[3]);

        ElapsedTime hi = new ElapsedTime();

        while(hi.milliseconds() <= 6000){

            telemetry.addData("Target Angle", storage.get(j)[4]);
            telemetry.addData("power BR", save4);
            telemetry.addData("power BL", save2);
            telemetry.addData("power FR", save3);
            telemetry.addData("power FL", save1);

            telemetry.addData("power BR", rightBack.getPower());
            telemetry.addData("power BL", leftBack.getPower());
            telemetry.addData("power FR", rightFront.getPower());
            telemetry.addData("power FL", leftFront.getPower());
            telemetry.update();
        }


    }

    //Whenever the robot needs turns or snapbot, always reset encoders to the direction they were intialized before
    public void resetEncoders() throws InterruptedException{
        for (DcMotorEx motor : motors) {
            if (motor == leftFront || motor == leftBack)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    //Returns the current angle of the robot calculated by the IMU
    public double currentAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void correction(double power, double targetHeading, String movementType, boolean inverted, double max) {
        //sets target and current angles
        double target = targetHeading;
        double current = currentAngle();

        //if the spline motion is backwards, the target must be flipped 180 degrees in order to match with spline.getAngle()
        if (inverted && movementType.contains("spline")) {
            target = (targetHeading > 0) ? (targetHeading - 180) : (targetHeading + 180);
        }

        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        double error = getError(current, target);

        //PD correction for both regular and spline motion
        if (movementType.contains("straight") || movementType.contains("spline")) {
            double correction = forwardPID.getCorrection(error, runtime);

            double leftPower = Range.clip(power - correction, -max, max);
            double rightPower = Range.clip(power + correction, -max, max);
            double nearing = 0;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);
//            telemetry.addData("left expected power", leftPower);
//            telemetry.addData("right expected power", rightPower);
//            telemetry.addData("actual left power", leftFront.getPower());
//            telemetry.addData("actual right power", rightFront.getPower());
        }

        //pd correction for strafe motion. Right and left are opposites

        else if (movementType.contains("strafe")) {
            double correction = strafePID.getCorrection(error, runtime);

            if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(-power + correction, -1.0, 1.0));
            } else if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(-power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(power + correction, -1.0, 1.0));
            }
        }

//        telemetry.addData("current Angle", current);
//        telemetry.addData("target", target);
//        telemetry.addData("error", error);
//        telemetry.addData("lf", leftFront.getPower());
//        telemetry.addData("lb", leftBack.getPower());
//        telemetry.addData("rf", rightFront.getPower());
//        telemetry.addData("rb", rightBack.getPower());
//        telemetry.addData("avg power", (rightBack.getPower() + rightFront.getPower() + leftBack.getPower() + leftFront.getPower()) / 4);
//        telemetry.update();
    }
    public double getError(double current, double target) {
        double error;

        double error1 = /*current - target*/ target - current;
        double error2;
        if (current < 0)
            error2 = /*(current + 360) - (target);*/ target - (current + 360);
        else
            error2 = /*(current - 360) - (target);*/ target - (current - 360);
        if (Math.abs(error1) <= Math.abs(error2))
            error = error1;
        else
            error = error2;

        return error;
    }


    public void print() throws InterruptedException{
        ElapsedTime aman = new ElapsedTime();

        ArrayList<Double> errors = forwardPID.aman();

        for(int i = 0; i < errors.size(); i++){
            telemetry.addData("Error " + i, errors.get(i));
        }
        telemetry.update();
        pause(10000);

    }

    public void powerShot() throws InterruptedException {
        double pow = -.75;
        //snapBot();
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setPower(pow);
        //Target position for shooting
        double targetX = 26.5;
        double targetY = 50.5;
        double initialDistance = 66;
        double powerIncrement = 0.06;
        snapBot();
        yeetLessRing(pow);

        pause(500);
        //strafeByWheelEncoders(0, 8, .4, "strafe right");
        turn(Math.toDegrees(Math.atan2(7.5, initialDistance)));
        yeetLessRing(pow-.1);//-powerIncrement);
        pause(500);
        //strafeByWheelEncoders(0, 8, .4, "strafe right");
        turn(Math.toDegrees(Math.atan2(15,initialDistance))-Math.toDegrees(Math.atan2(7.5,initialDistance)));
        yeetLessRing(pow-.08);
    }

    //The robot pauses for a certain amount of milliseconds
    public void pause(int time){
        ElapsedTime bruh = new ElapsedTime();
        while(bruh.milliseconds()<time){};
    }

    //The robot intakes rings while driving backwards
    public void intake(double pow,boolean zoneC) throws InterruptedException {
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(zoneC){
            intake.setPower(-7);
            transfer.setPower(.6);
            moveByWheelEncoders(0,15,-.075,"straight");

        }
        else{
            for (DcMotorEx motor : motors) {
                motor.setTargetPosition((int) (TPI * 10));
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setPower(pow);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(-1);
                transfer.setPower(1);
            }

        }
        ElapsedTime succ = new ElapsedTime();
        while (succ.milliseconds() < (zoneC?2000:2000))
            heartbeat();
        intake.setPower(0);
        transfer.setPower(0);
    }

    //The robot shoots rings (3 rings) from the ring stack
    public void yeetRing(double pow) throws InterruptedException {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(pow - .12);
        ElapsedTime shooterTime = new ElapsedTime();
        int flick = 2000;
        while (shooterTime.milliseconds() <= 5000) {
            flick = (int) shooterTime.milliseconds() + flick;
            if (shooterTime.milliseconds() >= 2000) {
                while (shooterTime.milliseconds() <= flick)
                    flicker.setPosition(0);
                shooter.setPower(pow-.09);
            }
            flick = (int) shooterTime.milliseconds() + 500;
            while (shooterTime.milliseconds() <= flick)
                flicker.setPosition(.7);
            flick = 800;

        }
        flicker.setPosition(.7);
        intake.setPower(0);
        transfer.setPower(0);
        shooter.setPower(0);
    }

    //The robot shoots ONLY one ring (this is mainly used for the Powershots)
    public void yeetLessRing(double pow) throws InterruptedException {
        shooter.setPower(pow);
        ElapsedTime shooterTime = new ElapsedTime();
        while (shooterTime.milliseconds() <= 500) {
            if (shooterTime.milliseconds() <= 250) flicker.setPosition(0);
            else {
                flicker.setPosition(.7);
                shooter.setPower(0);
            }
        }
    }



    public void resetMotors() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }

    //The robot's arm moves down and goes back up
    //Can be used to pick the wobbler or put down the wobbler
    public void armTravel() throws InterruptedException {
        ElapsedTime wobbler = new ElapsedTime();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(-2268);

        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (wobbler.milliseconds() <= 3000) {
            heartbeat();
            if (!arm.isBusy()) {
                claw.setPosition(0);
                break;
            }
        }
        arm.setTargetPosition(2268);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(-1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy()) {
            heartbeat();
        }
        arm.setPower(0);

    }



    public void turn(double turnAmount) throws InterruptedException {//right is negative sdlijhfkjdhfjklashdflkasdhjklfahsdjklfhasjkldfhlasjdkhfjkasfdhlk
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetEncoders();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        turnAmount=-turnAmount;
        double startAngle = angles.firstAngle;
        double desiredAngle = startAngle+turnAmount;
        if(desiredAngle>=180) desiredAngle=-180+desiredAngle%180;
        else if(desiredAngle<=-180) desiredAngle=180-(-desiredAngle%180);
        int turnFactor = (int) (turnAmount/Math.abs(turnAmount));// (turnAmount/Math.abs(turnAmount) determines if right or left. if left this value will be -1 and swap power values
        double initialPower;
        double minSpeed;
        if(Math.abs(turnAmount)<=85&&Math.abs(angles.firstAngle)>45){
            initialPower=.5;
            minSpeed=.15;
        }
        else if(Math.abs(turnAmount)<=45){
            initialPower=.4;
            minSpeed=.1;
        }
        else{
            initialPower=1;
            minSpeed=.4;
        }

//        telemetry.addData("RF",rightFront.getPower());
//        telemetry.addData("LB",leftBack.getPower());
//        telemetry.addData("RB",rightBack.getPower());
        telemetry.addData("desired angle change", turnAmount);
        telemetry.addData("current angle", angles.firstAngle);
        telemetry.addData("start angle", startAngle);
        telemetry.addData("degrees turned", Math.abs(angles.firstAngle-startAngle));
        telemetry.addData("desired angle", desiredAngle);
        telemetry.addData("distance to desired", Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle)));
        telemetry.addData("initial power", initialPower);
        telemetry.addData("current power", leftFront.getPower());
        telemetry.update();

        leftFront.setPower(-initialPower*turnFactor);
        rightFront.setPower(initialPower*turnFactor);
        leftBack.setPower(-initialPower*turnFactor);
        rightBack.setPower(initialPower*turnFactor);

        while((int)Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle))>5){ // (angles.firstAngle-startAngle-Math.abs(turnAmount)) is the difference between current angle and desired. Closer to desired angle = lower value.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            leftFront.setPower(-Range.clip(Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle))/Math.abs(turnAmount),minSpeed,initialPower)*turnFactor);
            rightFront.setPower(Range.clip(Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle))/Math.abs(turnAmount),minSpeed,initialPower)*turnFactor);
            leftBack.setPower(-Range.clip(Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle))/Math.abs(turnAmount),minSpeed,initialPower)*turnFactor);
            rightBack.setPower(Range.clip(Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle))/Math.abs(turnAmount),minSpeed,initialPower)*turnFactor);


//            telemetry.addData("LF",leftFront.getPower());
//            telemetry.addData("RF",rightFront.getPower());
//            telemetry.addData("LB",leftBack.getPower());
//            telemetry.addData("RB",rightBack.getPower());
            telemetry.addData("desired angle change", turnAmount);
            telemetry.addData("current angle", angles.firstAngle);
            telemetry.addData("start angle", startAngle);
            telemetry.addData("degrees turned", Math.abs(angles.firstAngle-startAngle));
            telemetry.addData("desired angle", desiredAngle);
            telemetry.addData("distance to desired", Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle)));
            telemetry.addData("initial power", initialPower);
            telemetry.addData("current power", leftFront.getPower());
            telemetry.update();
            heartbeat();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
//        ElapsedTime wait = new ElapsedTime();
//        while(wait.milliseconds()<5000){
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            telemetry.addData("desired angle change", turnAmount);
//            telemetry.addData("current angle", angles.firstAngle);
//            telemetry.addData("start angle", startAngle);
//            telemetry.addData("degrees turned", Math.abs(angles.firstAngle-startAngle));
//            telemetry.addData("desired angle", desiredAngle);
//            telemetry.addData("distance to desired", Math.abs(Math.abs(angles.firstAngle)-Math.abs(desiredAngle)));
//            telemetry.addData("initial power", initialPower);
//            telemetry.addData("current power", leftFront.getPower());
//            telemetry.update();
//            heartbeat();
//        }
    }

    //----------------------------------------------------------------------------
    //End of important code
    //This is only for OpenCV. Does nothing more than detect the rings
    public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline {

        public enum RingPosition{
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90, 80);

        static final int REGION_WIDTH = 10;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 160;
        final int ONE_RING_THRESHOLD = 140;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile RingPosition position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;

        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            if(avg1 > FOUR_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            } else if(avg1 > ONE_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.ONE;
            } else{
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
        public RingPosition getPosition() {
            return position;
        }


    }



}
