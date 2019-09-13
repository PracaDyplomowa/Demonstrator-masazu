package application;

import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.handGuiding;

import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.ioModel.AbstractIO;

import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineJP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

public class Massage extends RoboticsAPIApplication {

	public static enum ProgramMode {
		LIN, ROTATE, CIRCLE, END, NOTHING
	};

	private ProgramMode mode;
	private LBR lbr;
	private MediaFlangeIOGroup mediaFlange;
	private Controller kuka_Sunrise_Cabinet_1;
	private AbstractIO greenButton;
	private BooleanIOCondition greenButton_active;
	private Tool thumbTool;

	private boolean updateFlag = false, stiffnessFlag = true,
			modeChangeFlag = false, radiusChangingFlag = false;
	private boolean perpFlag;
	private boolean userButton;

	private CartesianImpedanceControlMode impedanceControlMode;
	private double stiffnessT = 2000;
	private double stiffnessR = 200;

	// upDown movement
	private SplineJP upDownMotion;
	private Frame upMotionFrame, downMotionFrame;
	private double upDownMotionPositionZ;

	// circle movement
	private Spline circleMotion;
	private Frame circleMotionPoint1Frame, circleMotionPoint2Frame,
	circleMotionPoint3Frame, circleMotionPoint4Frame;
	private double circlePoint2PositionX;
	private double circlePoint2PositionY;
	private double circlePoint3PositionY;
	private double circlePoint4PositionX;
	private double circlePoint4PositionY;

	// rotate movement
	private SplineJP rotateMotion;
	private Frame rotateMovePoint1Frame, rotateMovePoint2Frame;
	private double rotateAngleAofR1;

	private double velocity = 0.1;
	private double radius = 10;

	private void setMotion(ProgramMode mode){

		if(mode == ProgramMode.LIN){

			upDownMotionPositionZ = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame")).getZ();

			upMotionFrame = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame"));
			downMotionFrame = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame"));

			downMotionFrame = downMotionFrame.setZ(upDownMotionPositionZ - radius);

			upDownMotion = new SplineJP (ptp(upMotionFrame), ptp(downMotionFrame));

		}

		if(mode == ProgramMode.CIRCLE){


			circleMotionPoint1Frame = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame"));
			circleMotionPoint2Frame = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame"));
			circleMotionPoint3Frame = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame"));
			circleMotionPoint4Frame = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame"));

			circlePoint2PositionX = circleMotionPoint2Frame.getX();
			circlePoint2PositionY = circleMotionPoint2Frame.getY();

			circlePoint3PositionY = circleMotionPoint3Frame.getY();

			circlePoint4PositionX = circleMotionPoint4Frame.getX();
			circlePoint4PositionY = circleMotionPoint4Frame.getY();

			circleMotionPoint2Frame.setX(circleMotionPoint2Frame.getX() - radius);
			circleMotionPoint2Frame.setY(circleMotionPoint2Frame.getY() - radius);
			circleMotionPoint3Frame.setY(circleMotionPoint3Frame.getY() - 2*radius);
			circleMotionPoint4Frame.setX(circleMotionPoint4Frame.getX() + radius);
			circleMotionPoint4Frame.setY(circleMotionPoint4Frame.getY() - radius);

			circleMotion = new Spline(circ(circleMotionPoint2Frame, circleMotionPoint3Frame),circ(circleMotionPoint4Frame, circleMotionPoint1Frame));
		}


		if(mode == ProgramMode.ROTATE){

			JointPosition axisValueRad = lbr.getCurrentJointPosition();

			rotateMovePoint1Frame = lbr.getCurrentCartesianPosition(thumbTool.getFrame("/thumbFrame"));

			rotateMotion = new SplineJP(ptp(rotateMovePoint1Frame)
					,ptp(axisValueRad.get(0),axisValueRad.get(1),axisValueRad.get(2),axisValueRad.get(3)
							,axisValueRad.get(4),axisValueRad.get(5),axisValueRad.get(6) + Math.toRadians(radius)));

		}

	}

	private void updateMotionRadius() {

		if (mode == ProgramMode.LIN) {
			downMotionFrame = downMotionFrame.setZ(upDownMotionPositionZ
					- radius);
		}

		if (mode == ProgramMode.CIRCLE) {
			circleMotionPoint2Frame.setX(circlePoint2PositionX - radius);
			circleMotionPoint2Frame.setY(circlePoint2PositionY - radius);
			circleMotionPoint3Frame.setY(circlePoint3PositionY - 2 * radius);
			circleMotionPoint4Frame.setX(circlePoint4PositionX + radius);
			circleMotionPoint4Frame.setY(circlePoint4PositionY - radius);
		}

		if (mode == ProgramMode.ROTATE) {
			rotateMovePoint2Frame.setAlphaRad(rotateAngleAofR1
					+ Math.toRadians(radius));
		}

		updateFlag = false;
	}

	public void initialize() {

		lbr = getContext().getDeviceFromType(LBR.class);
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1"); // inicjalizacja kontrolera

		mediaFlange = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		mediaFlange.setLEDBlue(false);

		greenButton = mediaFlange.getInput("UserButton");
		greenButton_active = new BooleanIOCondition(greenButton, true);

		thumbTool = getApplicationData().createFromTemplate("kciukTool");
		thumbTool.attachTo(lbr.getFlange());

		mode = ProgramMode.NOTHING;

		impedanceControlMode = new CartesianImpedanceControlMode();

		IUserKeyBar userBar = getApplicationUI().createUserKeyBar("Param"); // stworzenie
		// paska
		// uzytkownika
		IUserKeyBar stiffnessBar = getApplicationUI().createUserKeyBar(
				"Sztywnoœæ"); // pasek zmiany sztywnosci
		IUserKeyBar moveBar = getApplicationUI().createUserKeyBar("Ruch"); // pasek
		// zmiany
		// wzoru
		// ruchu

		// LISTENER SZYBKOSCI

		IUserKeyListener listSpeedUp = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					velocity += 0.1;
					if (velocity >= 0.5)
						velocity = 0.5;
					getLogger().info(
							"Prêdkoœc wzglêdna: "
									+ Double.toString(Math
											.round(velocity * 100)) + " %");
				}
			}
		};

		IUserKeyListener listSpeedDown = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					velocity -= 0.1;
					if (velocity <= 0.1)
						velocity = 0.1;
					getLogger().info(
							"Prêdkoœc wzglêdna: "
									+ Double.toString(Math
											.round(velocity * 100)) + " %");
				}
			}
		};

		//LISTENER PROMIENIA
		IUserKeyListener listPosUp = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					radius += 5;
					if (radius > 19) {
						radius = 20;
					}
					radiusChangingFlag = true;
					getLogger().info(
							"D³ugoœæ ruchu: " + Double.toString(radius));
				}

			}

		};
		IUserKeyListener listPosDown = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					radius -= 5;
					if (radius < 5) {
						radius = 5;
					}
					radiusChangingFlag = true;
					getLogger().info(
							"D³ugoœæ ruchu: " + Double.toString(radius));
				}
			}
		};

		// LISTENER ZWIEKSZANIA SZTYWNOSCI
		IUserKeyListener listStiffnesTUp = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown && !stiffnessFlag) {
					stiffnessFlag = true;
					if (stiffnessT <= 4800) {
						stiffnessT = stiffnessT + 200;
						getLogger().info(
								"Sztywnoœæ translacyjna w osiach X, Y, Z: "
										+ Double.toString(stiffnessT));
					}

				}
			}
		};
		// LISTENER ZMNIEJSZANIA SZTYWNOSCI
		IUserKeyListener listStiffnesTDown = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown && !stiffnessFlag) {
					stiffnessFlag = true;
					if (stiffnessT >= 1200) {
						stiffnessT = stiffnessT - 200;
						getLogger().info(
								"Sztywnoœæ translacyjna w osiach X, Y, Z: "
										+ Double.toString(stiffnessT));
					}
				}
			}
		};
		// LISTENER ZWIEKSZANIA SZTYWNOSCI
		IUserKeyListener listStiffnesRUp = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown && !stiffnessFlag) {
					stiffnessFlag = true;
					if (stiffnessR <= 280) {
						stiffnessR = stiffnessR + 20;
						getLogger().info(
								"Sztywnoœæ rotacyjna w osiach X, Y, Z: "
										+ Double.toString(stiffnessR));
					}

				}
			}
		};

		// LISTENER ZMNIEJSZANIA SZTYWNOSCI
		IUserKeyListener listStiffnesRDown = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown && !stiffnessFlag) {
					stiffnessFlag = true;
					if (stiffnessR >= 40) {
						stiffnessR = stiffnessR - 20;
						getLogger().info(
								"Sztywnoœæ rotacyjna w osiach X, Y, Z: "
										+ Double.toString(stiffnessR));
					}
				}
			}
		};

		// ***************************LISTENER RUCHU
		// UP/DOWN***********************************************************//

		IUserKeyListener listMoveUpDown = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					if (mode != ProgramMode.LIN) {
						radius = 10;
						modeChangeFlag = true;
						mode = ProgramMode.LIN;
						getLogger().info("Typ ruchu: Góra/Dó³");
					}
				}
			}
		};

		// LISTENER RUCHU KOLOWEGO
		IUserKeyListener listMoveCircle = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					if (mode != ProgramMode.CIRCLE) {
						radius = 10;
						modeChangeFlag = true;
						mode = ProgramMode.CIRCLE;
						getLogger().info("Typ ruchu: Ko³owy");
					}
				}
			}
		};

		// LISTENER RUCHU OBROTOWEGO
		IUserKeyListener listMoveRotate = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					if (mode != ProgramMode.ROTATE) {
						radius = 10;
						modeChangeFlag = true;
						mode = ProgramMode.ROTATE;
						getLogger().info("Typ ruchu: Obrotowy");
					}
				}
			}
		};

		// LISTENER KONCA PROGRAMU
		IUserKeyListener listEnd = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					// if( mode != ProgramMode.END)
					updateFlag = true;
					mode = ProgramMode.END;
					getLogger().info("Koniec programu.");
				}
			}
		};

		// PRZYCISKI DO PIERWSZEGO PASKA
		IUserKey key00 = userBar.addUserKey(0, listSpeedUp, true);
		IUserKey key01 = userBar.addUserKey(1, listSpeedDown, true);
		IUserKey key02 = userBar.addUserKey(2, listPosUp, true);
		IUserKey key03 = userBar.addUserKey(3, listPosDown, true);

		// PRZYCISKI DO DRUGIEGO PASKA
		IUserKey key11 = stiffnessBar.addUserKey(0, listStiffnesTUp, true);
		IUserKey key12 = stiffnessBar.addUserKey(1, listStiffnesTDown, true);

		IUserKey key13 = stiffnessBar.addUserKey(2, listStiffnesRUp, true);
		IUserKey key14 = stiffnessBar.addUserKey(3, listStiffnesRDown, true);

		// PRZYCISKI DO TRZECIEGO PASKA
		IUserKey key21 = moveBar.addUserKey(0, listMoveUpDown, true);
		IUserKey key22 = moveBar.addUserKey(1, listMoveCircle, true);
		IUserKey key23 = moveBar.addUserKey(2, listMoveRotate, true);
		IUserKey key24 = moveBar.addUserKey(3, listEnd, true);

		// OPIS PRZYCISKOW NA PASKACH

		key00.setText(UserKeyAlignment.Middle, "V+");
		key01.setText(UserKeyAlignment.Middle, "V-");
		key02.setText(UserKeyAlignment.Middle, "R+");
		key03.setText(UserKeyAlignment.Middle, "R-");

		key11.setText(UserKeyAlignment.Middle, "Trans+");
		key12.setText(UserKeyAlignment.Middle, "Trans-");
		key13.setText(UserKeyAlignment.Middle, "Rot+");
		key14.setText(UserKeyAlignment.Middle, "Rot-");

		key21.setText(UserKeyAlignment.Middle, "Góra/Dó³");
		key22.setText(UserKeyAlignment.Middle, "Ko³o");
		key23.setText(UserKeyAlignment.Middle, "Obrót");
		key24.setText(UserKeyAlignment.Middle, "Koniec");

		userBar.publish();
		stiffnessBar.publish();
		moveBar.publish();

	}

	private void questionsAtStart() {

		int firstQuestion = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Czy chcesz rozpocz¹æ program?", "Tak", "Nie");

		switch (firstQuestion) {
		case 0:
			getLogger().info("Pocz¹tek programu");
			break;
		case 1:
			getLogger().info("Program przerwany");
			return;
		}

		int startDecision = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Czy chcesz dokonaæ bazowania?", "Tak", "Nie");

		if (startDecision == 0) {
			thumbTool.move(ptp(getApplicationData().getFrame("/BaseFrame"))
					.setJointVelocityRel(0.20));
		}
	}

	public void run() {

		questionsAtStart();

		thumbTool.move(handGuiding());

		while (true) {
			userButton = mediaFlange.getUserButton();

			if (mode == ProgramMode.LIN && modeChangeFlag) {
				setMotion(mode);
				modeChangeFlag = false;
			}

			if ((mode == ProgramMode.CIRCLE && modeChangeFlag)) {
				setMotion(mode);
				modeChangeFlag = false;
			}

			if (mode == ProgramMode.ROTATE && modeChangeFlag) {
				setMotion(mode);
				modeChangeFlag = false;
			}

			if (radiusChangingFlag) {
				updateMotionRadius();
				radiusChangingFlag = false;
			}

			if (stiffnessFlag) {
				impedanceControlMode.parametrize(CartDOF.TRANSL).setStiffness(
						stiffnessT);
				impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(
						stiffnessR);
				stiffnessFlag = false;
			}

			switch (mode) {
			case LIN:
				thumbTool.move(upDownMotion.setMode(impedanceControlMode)
						.setJointVelocityRel(velocity)
						.breakWhen(greenButton_active));
				break;
			case CIRCLE:
				thumbTool.move(circleMotion.setMode(impedanceControlMode)
						.setJointVelocityRel(velocity)
						.breakWhen(greenButton_active));
				break;
			case ROTATE:
				thumbTool.move(rotateMotion.setMode(impedanceControlMode)
						.setJointVelocityRel(velocity)
						.breakWhen(greenButton_active));
				break;
			case END:
				return;
			}

			while (userButton) {

				mediaFlange.setLEDBlue(true);
				thumbTool.move(handGuiding());
				mediaFlange.setLEDBlue(false);

				updateFlag = true;

				getLogger()
				.info("Wciœnij zielony przycisk w celu ustawienia efektora prostopadle do pod³o¿a.");

				userButton = false;

				for (int i = 0; i < 201; i++) {
					ThreadUtil.milliSleep(10);

					userButton = mediaFlange.getUserButton();

					if (userButton) {
						perpFlag = true;
						getLogger().info("Efektor ustawiany prostopadle.");
						break;
					}
				}

				if (perpFlag) { // przypisanie prostopadlego polozenia efektora
					// i ruch ptp zmieniajacy tylko katy, pozycja
					// xyz bez zmian
					Frame perpendicularFrame = lbr
							.getCurrentCartesianPosition(thumbTool
									.getFrame("/thumbFrame"));

					// perpFrame.setAlphaRad(perpFrame.getAlphaRad()); // k¹t
					// alfa wziêty z obecnej ramki
					perpendicularFrame.setBetaRad(Math.toRadians(0.0));
					perpendicularFrame.setGammaRad(Math.toRadians(-180));

					thumbTool.move(ptp(perpendicularFrame).setJointVelocityRel(
							0.2).setMode(impedanceControlMode));
					perpFlag = false;
				}

				getLogger()
				.info("Ruch rozpocznie siê za 2 sekundy, wciœnij zielony przycisk w celu ponownej aktywacji prowadzenia rêcznego.");
				userButton = false;

				// mediaFlange.setLEDBlue(true);

				for (int i = 0; i < 201; i++) { // mozliwosc zmiany
					ThreadUtil.milliSleep(10);
					if (i % 100 == 0)
						getLogger().info(Integer.toString(i / 100) + "s");

					userButton = mediaFlange.getUserButton();

					if (userButton) {

						getLogger().info("Prowadzenie rêczne.");
						break;
					}

				}

				setMotion(mode);
			}
		}
	}
}
