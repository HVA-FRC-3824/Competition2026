package frc.robot.lib.controls;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class Controller {
  
  private XboxController controller;

  public Controller(int id) {
    controller = new XboxController(id);
  }

  public AxisInput getAxisInput() {
    return new AxisInput(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), controller.getRightY());
  }

  public Button a()        { return new Button(Buttons.A, controller); }
  public Button b()        { return new Button(Buttons.B, controller); }
  public Button x()        { return new Button(Buttons.X, controller); }
  public Button y()        { return new Button(Buttons.Y, controller); }
  public Button leftBumper()     { return new Button(Buttons.LeftBumper, controller); }
  public Button rightBumper()    { return new Button(Buttons.RightBumper, controller); }
  public Button back()       { return new Button(Buttons.Back, controller); }
  public Button start()      { return new Button(Buttons.Start, controller); }
  public Button leftStickButton()  { return new Button(Buttons.LeftStickButton, controller); }
  public Button rightStickButton() { return new Button(Buttons.RightStickButton, controller); }
  public Button pov0()       { return new Button(Buttons.Pov_0, controller); }
  public Button pov45()      { return new Button(Buttons.Pov_45, controller); }
  public Button pov90()      { return new Button(Buttons.Pov_90, controller); }
  public Button pov135()       { return new Button(Buttons.Pov_135, controller); }
  public Button pov180()       { return new Button(Buttons.Pov_180, controller); }
  public Button pov225()       { return new Button(Buttons.Pov_225, controller); }
  public Button pov270()       { return new Button(Buttons.Pov_270, controller); }
  public Button pov315()       { return new Button(Buttons.Pov_315, controller); }

  public static enum Buttons {
    A         (  1),
    B         (  2),
    X         (  3),
    Y         (  4),
    LeftBumper    (  5),
    RightBumper   (  6),
    LeftTrigger   ( 99),
    RightTrigger  ( 99),
    Back      (  7),
    Start       (  8),
    LeftStickButton (  9),
    RightStickButton( 10),
    Pov_0       (  0),
    Pov_45      ( 45),
    Pov_90      ( 90),
    Pov_135     (135),
    Pov_180     (180),
    Pov_225     (225),
    Pov_270     (270),
    Pov_315     (315);

    public int m_button;

    private Buttons(int button) {
      m_button = button;
    }
  }

  public static class Button {

    XboxController m_controller;

    Buttons m_button;

    public Button(Buttons button, XboxController controller) {
      m_button = button;
      m_controller = controller;
    }

    public Button onPress(Command runner) {
      return onPress(runner::execute);
    }

    public Button onPress(Runnable runner) {
      switch (m_button) {
        case A: { if (m_controller.getAButtonPressed()) runner.run(); break;}
        case B: { if (m_controller.getBButtonPressed()) runner.run(); break;}
        case X: { if (m_controller.getXButtonPressed()) runner.run(); break;}
        case Y: { if (m_controller.getYButtonPressed()) runner.run(); break;}
        case LeftBumper: { if (m_controller.getLeftBumperButtonPressed()) runner.run(); break;}
        case RightBumper: { if (m_controller.getRightBumperButtonPressed()) runner.run(); break;}
        // case LeftTrigger:  { if (m_controller.getLeftTriggerAxis() >= 0.5) runner.run(); break;} TODO: Support
        // case RightTrigger: { if (m_controller.getRightTriggerAxis() >= 0.5) runner.run(); break;}
        case Back: { if (m_controller.getBackButtonPressed()) runner.run(); break;}
        case Start: { if (m_controller.getStartButtonPressed()) runner.run(); break;}
        case LeftStickButton: { if (m_controller.getLeftStickButtonPressed()) runner.run(); break;}
        case RightStickButton: { if (m_controller.getRightStickButtonPressed()) runner.run(); break;}
        case Pov_0: { if (m_controller.getPOV() == 0) runner.run(); break;}
        case Pov_45: { if (m_controller.getPOV() == 45) runner.run(); break;}
        case Pov_90: { if (m_controller.getPOV() == 90) runner.run(); break;}
        case Pov_135: { if (m_controller.getPOV() == 135) runner.run(); break;}
        case Pov_180: { if (m_controller.getPOV() == 180) runner.run(); break;}
        case Pov_225: { if (m_controller.getPOV() == 225) runner.run(); break;}
        case Pov_270: { if (m_controller.getPOV() == 279) runner.run(); break;}
        case Pov_315: { if (m_controller.getPOV() == 315) runner.run(); break;}
        default: {Logger.recordOutput("Button Unsupported", m_button.name());}
      }
      return this;
    }
    
    public Button onRelease(Command runner) {
      return onRelease(runner::execute);
    }

    public Button onRelease(Runnable runner) {
      switch (m_button) {
        case A: { if (m_controller.getAButtonReleased()) runner.run(); break;}
        case B: { if (m_controller.getBButtonReleased()) runner.run(); break;}
        case X: { if (m_controller.getXButtonReleased()) runner.run(); break;}
        case Y: { if (m_controller.getYButtonReleased()) runner.run(); break;}
        case LeftBumper: { if (m_controller.getLeftBumperButtonReleased()) runner.run(); break;}
        case RightBumper: { if (m_controller.getRightBumperButtonReleased()) runner.run(); break;}
        // case LeftTrigger:  { if (m_controller.getLeftTriggerAxis() >= 0.5) runner.run(); break;} TODO: Support
        // case RightTrigger: { if (m_controller.getRightTriggerAxis() >= 0.5) runner.run(); break;}
        case Back: { if (m_controller.getBackButtonReleased()) runner.run(); break;}
        case Start: { if (m_controller.getStartButtonReleased()) runner.run(); break;}
        case LeftStickButton: { if (m_controller.getLeftStickButtonReleased()) runner.run(); break;}
        case RightStickButton: { if (m_controller.getRightStickButtonReleased()) runner.run(); break;}
        case Pov_0: { if (m_controller.getPOV() == 0) runner.run(); break;}
        case Pov_45: { if (m_controller.getPOV() == 45) runner.run(); break;}
        case Pov_90: { if (m_controller.getPOV() == 90) runner.run(); break;}
        case Pov_135: { if (m_controller.getPOV() == 135) runner.run(); break;}
        case Pov_180: { if (m_controller.getPOV() == 180) runner.run(); break;}
        case Pov_225: { if (m_controller.getPOV() == 225) runner.run(); break;}
        case Pov_270: { if (m_controller.getPOV() == 279) runner.run(); break;}
        case Pov_315: { if (m_controller.getPOV() == 315) runner.run(); break;}
        default: {Logger.recordOutput("Button Unsupported", m_button.name() + " " + m_button.ordinal());}
      }
      return this;
    }

    public Button onTrue(Command runner) {
      return onTrue(runner::execute);
    }

    public Button onTrue(Runnable runner) {
      switch (m_button) {
        case A: { if (m_controller.getAButton()) runner.run(); break;}
        case B: { if (m_controller.getBButton()) runner.run(); break;}
        case X: { if (m_controller.getXButton()) runner.run(); break;}
        case Y: { if (m_controller.getYButton()) runner.run(); break;}
        case LeftBumper: { if (m_controller.getLeftBumperButton()) runner.run(); break;}
        case RightBumper: { if (m_controller.getRightBumperButton()) runner.run(); break;}
        case LeftTrigger:  { if (m_controller.getLeftTriggerAxis() >= 0.5) runner.run(); break;}
        case RightTrigger: { if (m_controller.getRightTriggerAxis() >= 0.5) runner.run(); break;}
        case Back: { if (m_controller.getBackButton()) runner.run(); break;}
        case Start: { if (m_controller.getStartButton()) runner.run(); break;}
        case LeftStickButton: { if (m_controller.getLeftStickButton()) runner.run(); break;}
        case RightStickButton: { if (m_controller.getRightStickButton()) runner.run(); break;}
        case Pov_0: { if (m_controller.getPOV() == 0) runner.run(); break;}
        case Pov_45: { if (m_controller.getPOV() == 45) runner.run(); break;}
        case Pov_90: { if (m_controller.getPOV() == 90) runner.run(); break;}
        case Pov_135: { if (m_controller.getPOV() == 135) runner.run(); break;}
        case Pov_180: { if (m_controller.getPOV() == 180) runner.run(); break;}
        case Pov_225: { if (m_controller.getPOV() == 225) runner.run(); break;}
        case Pov_270: { if (m_controller.getPOV() == 279) runner.run(); break;}
        case Pov_315: { if (m_controller.getPOV() == 315) runner.run(); break;}
        default: {Logger.recordOutput("Button Unsupported", m_button.name() + " " + m_button.ordinal());}
      }
      return this;
    }

  }
}