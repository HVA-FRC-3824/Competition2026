package frc.robot.lib.controls;

import edu.wpi.first.wpilibj.XboxController;

public class EdgeController {
  
  public static enum Edge {
    Rising,
    Falling
  }

  XboxController m_controller;

  boolean m_povUpLastInput    = false;
  boolean m_povDownLastInput  = false;
  boolean m_povLeftLastInput  = false;
  boolean m_povRightLastInput = false;

  public EdgeController(int port) {
    m_controller = new XboxController(port);
  }

  public AxisInput getAxisInput(double translateExpo, double rotationExpo) {
    // Read raw stick values
    double rawLeftX = m_controller.getLeftX();
    double rawLeftY = m_controller.getLeftY();
    double leftAngle = Math.atan2(rawLeftY, rawLeftX);
    double leftMagnitude = Math.hypot(rawLeftX, rawLeftY);

    // Apply translation expo to the magnitude only, preserving direction via the angle
    if (leftMagnitude > 0.0) {
      leftMagnitude = Math.pow(leftMagnitude, translateExpo);
    }

    // X = cos(angle) * mag, Y = sin(angle) * mag
    double leftX = leftMagnitude * Math.cos(leftAngle);
    double leftY = leftMagnitude * Math.sin(leftAngle);

    double rawRightX = m_controller.getRightX();
    double rawRightY = m_controller.getRightY();
    double rightAngle = Math.atan2(rawRightY, rawRightX);
    double rightMagnitude = Math.hypot(rawRightX, rawRightY);

    // Rotation stick should use rotationExpo
    if (rightMagnitude > 0.0) {
      rightMagnitude = Math.pow(rightMagnitude, rotationExpo);
    }

    double rightX = rightMagnitude * Math.cos(rightAngle);
    double rightY = rightMagnitude * Math.sin(rightAngle);

    return new AxisInput(leftX, leftY, rightX, rightY);
  }

  public void a(Edge edge, Runnable routine)      {
    boolean check = false                         ;
    switch (edge)                                 {
      case Rising                                 :
        check = m_controller.getAButtonPressed()  ;
        break                                     ;
      case Falling                                :
        check = m_controller.getAButtonReleased() ;
        break                                     ;
                                                  }
    if (check && routine != null)                 {
      routine.run()                               ;
                                                  }
                                                  }
                                  
  public void b(Edge edge, Runnable routine)      {
    boolean check = false                         ;
    switch (edge)                                 {
      case Rising                                 :
        check = m_controller.getBButtonPressed()  ;
        break                                     ;
      case Falling                                :
        check = m_controller.getBButtonReleased() ;
        break                                     ;
                                                  }
    if (check && routine != null)                 {
      routine.run()                               ;
                                                  }
                                                  }
                                  
  public void x(Edge edge, Runnable routine)            {
    boolean check = false                     ;
    switch (edge)                         {
      case Rising                       :
        check = m_controller.getXButtonPressed()      ;
        break                         ;
      case Falling                      :
        check = m_controller.getXButtonReleased() ;
        break                         ;
                                  }
    if (check && routine != null)                 {
      routine.run()                       ;
                                  }
                                  }
                                  
  public void y(Edge edge, Runnable routine)            {
    boolean check = false                     ;
    switch (edge)                         {
      case Rising                       :
        check = m_controller.getYButtonPressed()      ;
        break                         ;
      case Falling                      :
        check = m_controller.getYButtonReleased()       ;
        break                         ;
                                  }
    if (check && routine != null)                 {
      routine.run()                       ;
                                  }
                                  }
                                  
  public void leftBumper(Edge edge, Runnable routine)       {
    boolean check = false                     ;
    switch (edge)                         {
      case Rising                       :
        check = m_controller.getLeftBumperButtonPressed()   ;
        break                         ;
      case Falling                      :
        check = m_controller.getLeftBumperButtonReleased()  ;
        break                         ;
                                  }
    if (check && routine != null)                 {
      routine.run()                       ;
                                  }
                                  }
                                  
  public void rightBumper(Edge edge, Runnable routine)      {
    boolean check = false                     ;
    switch (edge)                         {
      case Rising                       :
        check = m_controller.getRightBumperButtonPressed()  ;
        break                         ;
      case Falling                      :
        check = m_controller.getRightBumperButtonReleased() ;
        break                         ;
                                  }
    if (check && routine != null)                 {
      routine.run()                       ;
                                  }
                                  }
                                  
  public void up(Edge edge, Runnable routine)           {
    boolean check = m_controller.getPOV() == 0          ;
    switch (edge)                         {
      case Rising                       :
        check = !m_povUpLastInput && check          ;
        break                         ;
      case Falling                      :
        check = m_povUpLastInput && !check          ;
        break                         ;
                                  }
    if (check && routine != null)                 {
      routine.run()                       ;
                                  }
    m_povUpLastInput = m_controller.getPOV() == 0         ;
                                  }

  public void down(Edge edge, Runnable routine)           {
    boolean check = m_controller.getPOV() == 180;
    switch (edge) {
      case Rising:
        check = !m_povDownLastInput && check;
        break;
      case Falling:
        check = m_povDownLastInput && !check;
        break;
    }
    if (check && routine != null)                 {
      routine.run()                       ;
                                  }
    m_povDownLastInput = m_controller.getPOV() == 180;
                                  }
                                  
  public void left(Edge edge, Runnable routine)           {
    boolean check = m_controller.getPOV() == 270;
    switch (edge) {
      case Rising:
        check = !m_povLeftLastInput && check;
        break;
      case Falling:
        check = m_povLeftLastInput && !check;
        break;
    }
    if (check && routine != null)                 {
      routine.run()                       ;
                                  }
    m_povLeftLastInput = m_controller.getPOV() == 270;
                                  }
                                  
  public void right(Edge edge, Runnable routine)          {
    boolean check = m_controller.getPOV() == 90;
    switch (edge) {
      case Rising:
        check = !m_povRightLastInput && check;
        break;
      case Falling:
        check = m_povRightLastInput && !check;
        break;
    }
    if (check && routine != null) {
      routine.run();
    }
    m_povRightLastInput = m_controller.getPOV() == 90;
                                  }
}
