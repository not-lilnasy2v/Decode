package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "O testare")
public class Tel extends OpMode {
    private DcMotorEx turela, frontRight, frontLeft, backRight, backLeft, shooter;
    private ServoImplEx Saruncare;
    private Limelight3A limelight3A;
    boolean stop;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double FL, BL, BR, FR;
    //PIDF
    private final double TkP = 0.008, TkI = 0.0, TkD = 0.08, SkP = 10.23, SkI = 0.0, SkF = 14.95, SkD = 10.58;
    private final int TURRET_MIN_POS = 309;
    private final int TURRET_MAX_POS = 647;
    private final int TURRET_INIT_POS = 490;
    public boolean taranie = false, abis = false, jay = false, Ungur = false, MiklosUngur = false;
    private double integral = 0, lastError = 0, tx = 0, power;
    int pos, posFR, posFL, posBR, posBL;


    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

//        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);


        turela = hardwareMap.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        do {
            pos = turela.getCurrentPosition();
            if (pos >= TURRET_INIT_POS) {
                turela.setPower(-0.03);
            } else if (pos <= TURRET_INIT_POS) {
                turela.setPower(0.03);
            }
        } while (pos == TURRET_INIT_POS);

        Saruncare = hardwareMap.get(ServoImplEx.class, "aruncare");

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();


    }

    public void start() {
        Chassis.start();
        System.start();
        Turela.start();
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                FL = y + x + rx;
                BL = y - x + rx;
                FR = y - x - rx;
                BR = y + x - rx;

                max = abs(FL);
                if (abs(FR) > max) {
                    max = abs(FR);
                }
                if (abs(BL) > max) {
                    max = abs(BL);
                }
                if (abs(BR) > max) {
                    max = abs(BR);
                }
                if (max > 1) {
                    FL /= max;
                    FR /= max;
                    BL /= max;
                    BR /= max;
                }
                if (gamepad1.left_trigger > 0) {
                    sm = 2;
                } else {
                    if (gamepad1.right_trigger > 0) {
                        sm = 5;
                    } else {
                        sm = 1;
                    }
                }

                POWER(FR / sm, BL / sm, BR / sm, FL / sm);

            }
        }
    });
    private final Thread System = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {


                /// Turela buton Trigger
                pos = turela.getCurrentPosition();
                boolean abut = gamepad1.y;
                if (taranie != abut) {
                    if (gamepad1.y) {
                        abis = !abis;
                    }
                    taranie = abut;
                }


                ///Servo Aruncare in shooter
                boolean african = gamepad1.x;
                if (jay != african) {
                    if (jay) {
                        Saruncare.setPosition(0.3665);
                    } else {
                        Saruncare.setPosition(0.621);
                    }
                    jay = african;
                }


                ///Shooter
                PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
                shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);

                boolean Miklos = gamepad1.a;
                if (Ungur != Miklos) {
                    if (gamepad1.a) {
                        MiklosUngur = !MiklosUngur;
                        if (MiklosUngur) {
                            shooter.setVelocity(2000);
                        } else {
                            shooter.setVelocity(0);
                        }
                    }
                    Ungur = Miklos;
                }


            }
        }
    });


    /// Ce trebuie sa faca turela
    private final Thread Turela = new Thread(new Runnable() {

        @Override
        public void run() {

            while (!stop && abis) {

                if ((pos >= TURRET_MIN_POS) || (pos <= TURRET_MAX_POS)) {

                    do {
                        LLResult result = limelight3A.getLatestResult();
                        if (result.isValid() && (pos >= TURRET_MIN_POS) || (pos <= TURRET_MAX_POS)) {
                            tx = result.getTx();
                            double error = tx;
                            integral += error;
                            double derivative = error - lastError;

                            power = TkP * error + TkI * integral + TkD * derivative;

                            turela.setPower(power);

                            lastError = error;

                        } else if (pos >= TURRET_MAX_POS && !result.isValid() || tx == 0)
                            power = 0.01;
                        else if (pos <= TURRET_MIN_POS && !result.isValid() || tx == 0)
                            power = 0.0;
                    } while (Math.abs(tx) > 0.05);
                }
                turela.setPower(0);

            }
        }
    });

    public void stop() {
        stop = true;
    }

    @Override
    public void loop() {
        telemetry.addData("tx", tx);
        telemetry.addData("pos", pos);
        telemetry.addData("power", power);
        telemetry.addData("ab", abis);
        telemetry.addData("jay", jay);
//        telemetry.addLine("==============");
//        telemetry.addData("fr", posFR);
//        telemetry.addData("fl", posFL);
//        telemetry.addData("br", posBR);
//        telemetry.addData("bl", posBL);
        telemetry.update();

    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}