package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Main")
public class Tel extends OpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft, turela, shooter, intake;
    private ServoImplEx Saruncare, sortare,unghiD,unghiS;
    private Limelight3A limelight3A;
    private ColorSensor color;
    private DistanceSensor distanta;
    boolean stop;
    double sm = 1;
    double max = 0;
    double FL, BL, BR, FR;
    sistemeTeleOp m = new sistemeTeleOp();

    //PIDF
    private final double TkP = 0.008, TkI = 0.0, TkD = 0.08, SkP = 10.23, SkI = 0.0, SkF = 14.95, SkD = 10.58;
    public boolean turelaTracking = false, tracking = false, ShooterPornit = false, Spornit = false, IntakeFull = false, Ipornit = false, IntakePornit = false, ButonulRosu = false, ResetareISortare = false, ResetareSSortare = false;
    private double integral = 0, lastError = 0, imata, tx = 0, power = 0, posU = Pozitii.max_sus;
    int pos, albastru = 0, rosu = 0, verde = 0, loculete = 0, idTag = 2;


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

//        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPower(0);


        turela = hardwareMap.get(DcMotorEx.class, "turela");
        turela.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turela.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Saruncare = hardwareMap.get(ServoImplEx.class, "aruncare");
        Saruncare.setPosition(Pozitii.coborare);
        sortare = hardwareMap.get(ServoImplEx.class, "sortare");
        sortare.setPosition(Pozitii.luarea1);
        unghiD = hardwareMap.get(ServoImplEx.class, "unghiD");
        unghiS = hardwareMap.get(ServoImplEx.class, "unghiS");


        distanta = hardwareMap.get(DistanceSensor.class, "distanta");


        color = hardwareMap.get(ColorSensor.class, "color");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setPower(0);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        unghiD.setPosition(posU);
        unghiS.setPosition(posU);

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }


    public void start() {
        Chassis.start();
        Butoane.start();
        Turela.start();
        Sortare.start();
        Shooter.start();
    }


    private final Thread Butoane = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                /// Turela toggle
                boolean dpad_right1 = gamepad1.dpad_right;
                if (turelaTracking != dpad_right1) {
                    if (gamepad1.dpad_right) {
                        tracking = !tracking;
                    }
                    turelaTracking = dpad_right1;
                }
                if(gamepad1.dpad_up){
                    posU += 0.003;
                }
                if(gamepad1.dpad_down){
                    posU -= 0.003;
                }
                if(posU <= Pozitii.max_jos && posU >= Pozitii.max_sus) {
                    unghiD.setPosition(posU);
                    unghiS.setPosition(posU);
                }
                /// Intake
                boolean gamepad1_a = gamepad1.a;
                if (IntakePornit != gamepad1_a) {
                    if (gamepad1.a) {
                        Ipornit = !Ipornit;
                    }
                    IntakePornit = gamepad1_a;
                }
                /// shooter
                boolean gamepad1_x = gamepad1.x;
                if (ShooterPornit != gamepad1_x) {
                    if (gamepad1.x) {
                        Spornit = !Spornit;
                    }
                    ShooterPornit = gamepad1_x;
                }


                /// Resetare Sortare shooter
                boolean gamepad2_b = gamepad2.b;
                if (ResetareSSortare != gamepad2_b) {
                    if (gamepad2.b) {
                        ButonulRosu = !ButonulRosu;
                    }
                    ResetareSSortare = gamepad2_b;
                }
            }
        }
    });

    private final Thread Turela = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                if (tracking) {
                    pos = turela.getCurrentPosition();
                    LLResult result = limelight3A.getLatestResult();
                    if (result.isValid() && pos >= Pozitii.TURRET_MIN_POS && pos <= Pozitii.TURRET_MAX_POS) {

                        tx = result.getTx();

                        integral += tx;
                        double derivative = tx - lastError;

                        power = TkP * tx + TkI * integral + TkD * derivative;
                        turela.setPower(power);

                        lastError = tx;

                    }
                } else {
                    turela.setPower(0);
                    lastError = 0;
                    integral = 0;
                }
            }
        }
    });
    private final Thread Sortare = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (Ipornit) {
                    if (loculete >= 0 && loculete <=3 && !IntakeFull) {
                        intake.setPower(1);
                        while (loculete <= 3) {
                            imata = distanta.getDistance(DistanceUnit.CM);

                            if (imata < 20 && loculete == 0) {
                                sortare.setPosition(Pozitii.luarea2);
                                loculete++;
                                m.kdf(950);
                                break;
                            }
                            if (imata < 20 && loculete == 1) {
                                sortare.setPosition(Pozitii.luarea3);
                                loculete++;
                                m.kdf(950);
                                break;
                            }
                            if (imata < 20 && loculete == 2) {
                                IntakeFull = true;
                                loculete++;
                                break;
                            }
                            if (loculete == 3) {
                                sortare.setPosition(Pozitii.aruncare1);
                                loculete++;
                                intake.setPower(0);
                                break;
                            }
                        }
                    }


                } else {
                    intake.setPower(0);

                }
            }
        }
    });
    private enum SortareShooter {
        SIDLE,
        Incarca,
        CautaSiImpuscai,
        GATA
    }

    private SortareShooter Sstare = SortareShooter.SIDLE;
    private int[] cPattern = new int[3];
    private boolean[] ocupat = new boolean[3];

    private final Thread Shooter = new Thread(new Runnable() {
        @Override
        public void run() {

            while (!stop) {

                if (!(Spornit && IntakeFull && loculete > 0 && loculete <= 3)) {
                    shooter.setVelocity(0);
                    Sstare = SortareShooter.SIDLE;
                    m.kdf(100);
                    continue;
                }

                shooter.setVelocity(2000);

                switch (Sstare) {

                    case SIDLE:
                        sortare.setPosition(Pozitii.aruncare1);
                        m.kdf(300);

                        for (int i = 0; i < 3; i++)
                            ocupat[i] = (i < loculete);

                        Sstare = SortareShooter.Incarca;
                        break;

                    case Incarca:
                        if (idTag == 3) {
                            cPattern[0] = 1;
                            cPattern[1] = 1;
                            cPattern[2] = 0;
                        } /// mov mov verde
                        else if (idTag == 2) {
                            cPattern[0] = 1;
                            cPattern[1] = 0;
                            cPattern[2] = 1;
                        } /// mov verde mov
                        else if (idTag == 1) {
                            cPattern[0] = 0;
                            cPattern[1] = 1;
                            cPattern[2] = 1;
                        } ///verde mov mov
                        else {
                            Sstare = SortareShooter.GATA;
                            break;
                        }

                        Sstare = SortareShooter.CautaSiImpuscai;
                        break;

                    case CautaSiImpuscai:

                        for (int step = 0; step < 3 && loculete > 0; step++) {

                            int need = cPattern[step];
                            boolean tras = false;

                            if (ocupat[0]) {
                                sortare.setPosition(Pozitii.aruncare1);
                                m.kdf(1000);

                                boolean mov = color.green() <= Pozitii.mov_verde;
                                boolean verde = !mov;

                                if ((need == 1 && mov) || (need == 0 && verde)) {
                                    Saruncare.setPosition(Pozitii.lansare);
                                    m.kdf(150);
                                    Saruncare.setPosition(Pozitii.coborare);

                                    ocupat[0] = false;
                                    loculete--;
                                    tras = true;
                                }
                            }
                            if (tras) continue;

                            if (ocupat[1]) {
                                sortare.setPosition(Pozitii.aruncare2);
                                m.kdf(1000);

                                boolean mov = color.green() <= Pozitii.mov_verde;
                                boolean verde = !mov;

                                if ((need == 1 && mov) || (need == 0 && verde)) {
                                    Saruncare.setPosition(Pozitii.lansare);
                                    m.kdf(150);
                                    Saruncare.setPosition(Pozitii.coborare);

                                    ocupat[1] = false;
                                    loculete--;
                                    tras = true;
                                }
                            }
                            if (tras) continue;

                            if (ocupat[2]) {
                                sortare.setPosition(Pozitii.aruncare3);
                                m.kdf(1000);

                                boolean mov = color.green() <= Pozitii.mov_verde;
                                boolean verde = !mov;

                                if ((need == 1 && mov) || (need == 0 && verde)) {
                                    Saruncare.setPosition(Pozitii.lansare);
                                    m.kdf(150);
                                    Saruncare.setPosition(Pozitii.coborare);

                                    ocupat[2] = false;
                                    loculete--;
                                    tras = true;
                                }
                            }
                            if (tras) continue;

                            int FB = -1;
                            if (ocupat[0]) FB = 0;
                            else if (ocupat[1]) FB = 1;
                            else if (ocupat[2]) FB = 2;

                            if (FB != -1) {
                                if (FB == 0) sortare.setPosition(Pozitii.aruncare1);
                                if (FB == 1) sortare.setPosition(Pozitii.aruncare2);
                                if (FB == 2) sortare.setPosition(Pozitii.aruncare3);

                                m.kdf(650);

                                Saruncare.setPosition(Pozitii.lansare);
                                m.kdf(150);

                                Saruncare.setPosition(Pozitii.coborare);

                                ocupat[FB] = false;
                                loculete--;
                            }
                        }

                        Sstare = SortareShooter.GATA;
                        break;

                    case GATA:
                        shooter.setVelocity(0);
                        IntakeFull = false;

                        if (ButonulRosu) {
                            IntakeFull = true;
                        }
                        Sstare = SortareShooter.SIDLE;
                        break;
                }
            }
        }
    });



    //                if (Spornit && IntakeFull && loculete > 0 && loculete <= 3) {
//                    PIDFCoefficients pid = new PIDFCoefficients(SkP, SkI, SkD, SkF);
//                    shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
//                    shooter.setVelocity(2000);
//                    switch (Sstare) {
//                        case SIDLE:
//                            sortare.setPosition(Pozitii.aruncare1);
//                            Sstare = SortareShooter.ID1;
//                            break;
//                        case ID1:
//                            if (idTag == 1) {
//
//
//                                Sstare = SortareShooter.Aruncare;
//
//                            } else {
//                                Sstare = SortareShooter.ID2;
//                                break;
//                            }
//                            break;
//                        case ID2:
//                            if (idTag == 2) {
//
//
//                                Sstare = SortareShooter.Aruncare;
//                            } else {
//                                Sstare = SortareShooter.ID3;
//                                break;
//                            }
//                            break;
//                        case ID3:
//                            if (idTag == 3) {
//                                if(verde <= Pozitii.mov_verde && loculete == 3 ) {
//                                    Saruncare.setPosition(Pozitii.lansare);
//                                    loculete--;
//                                    m.kdf(300);
//                                    Saruncare.setPosition(Pozitii.coborare);
//                                    sortare.setPosition(Pozitii.aruncare2);
//                                }
//                                else if(loculete==3 && verde >= Pozitii.mov_verde){
//                                    Saruncare.setPosition(Pozitii.aruncare2);
//                                }
//                                if(verde <= Pozitii.mov_verde && loculete == 2){
//                                    Saruncare.setPosition(Pozitii.lansare);
//                                    loculete--;
//                                    Saruncare.setPosition(Pozitii.coborare);
//                                    sortare.setPosition(Pozitii.aruncare3);
//                                }
//                                if(verde >= Pozitii.mov_verde && loculete == 1){
//                                    Saruncare.setPosition(Pozitii.lansare);
//                                    loculete--;
//                                    Saruncare.setPosition(Pozitii.coborare);
//                                    sortare.setPosition(Pozitii.aruncare1);
//                                }
//                                Sstare = SortareShooter.Aruncare;
//                            } else {
//                                Sstare = SortareShooter.Aruncare;
//                                break;
//                            }
//                            break;
//                        case Aruncare:
//
//                            Sstare = SortareShooter.Gol;
//                            break;
//                        case Gol:
//                            shooter.setVelocity(0);
//                            IntakeFull = false;
//                            Sstare = SortareShooter.SIDLE;
//                            if (ButonulRosuS) {
//                                IntakeFull = true;
//
//                            }
//                            break;
//                    }
//                } else {
//                    shooter.setVelocity(0);
//                }
//            }
//        }
//    });
    //                    do {
//                        shooter.setVelocity(2000);
//
//                        if (verde <= Pozitii.mov_verde) {
//                            m.kdf(980);
//                            Saruncare.setPosition(Pozitii.lansare);
//                            loculete--;
//                            m.kdf(2000);
//                            Saruncare.setPosition(Pozitii.coborare);
//
//
//                        } else if (verde != Pozitii.mov_verde) {
//                            sortare.setPosition(Pozitii.aruncare2);
//                        }
//                    } while (MiklosUngur && loculete >= 1 && loculete <= 3);
//            shooter.setVelocity(0);
//
//            sortare.setPosition(Pozitii.aruncare1);
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

    public void stop() {
        stop = true;
    }

    @Override
    public void loop() {
        telemetry.addData("Green", verde);
        telemetry.addData("Distanta", imata);
        telemetry.addData("loculete", loculete);
        telemetry.addData("Case", Sstare);
        telemetry.addData("pos", posU);
        telemetry.update();

    }

    public void POWER(double fr1, double bl1, double br1, double fl1) {
        frontRight.setPower(fr1);
        backLeft.setPower(bl1);
        frontLeft.setPower(fl1);
        backRight.setPower(br1);
    }
}