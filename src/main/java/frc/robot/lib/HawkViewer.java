package frc.robot.lib;
import java.util.ArrayList;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HawkViewer {
  // Build the markup
  @SuppressWarnings("unchecked") // Ima keep it a buck fiddy I have no clue why the compiler is so mad at me
  public static void testSampleMarkup() {

    JSONObject markup = new JSONObject();

    JSONArray elements = new JSONArray();

    // Helper to add widgets
    JSONObject element = new JSONObject();
    element.put("name", "Test");
    element.put("widget",  "Basic");
    element.put("source",  "/SmartDashboard/help");
    element.put("sizeX", 200.0);
    element.put("sizeY", 80.0);
    element.put("posX", 10.0);
    element.put("posY", 40.0);
    elements.add(element);

    markup.put("elements", elements);
    
    markup.put("name", "HawkViewer Test Dashboard");

    // Publish to NetworkTables
    SmartDashboard.putString("HawkViewerMarkup", markup.toJSONString());
  }

  private ArrayList<Widget> m_widgets = new ArrayList<>();

  // NOTE, THIS IS PURPOSEFULLY NOTTTTTTT STATIC
  // You are supposed to only have ***one*** instance of this
  public HawkViewer() {

  }

  public HawkViewer addChild(Widget widget) {
    m_widgets.add(widget);

    return this;
  }

  // ONLY CALL THIS ONCE
  public void pushMarkup() {
    String markup = createMarkupFromWidgets();

    NetworkTableInstance.getDefault().getStringTopic("HawkViewerMarkup")
      .genericPublish(markup, (PubSubOption[]) null);
  }

  @SuppressWarnings("unchecked")
  private String createMarkupFromWidgets() {
    JSONObject markup = new JSONObject();
    markup.put("name", "HawkViewer — Test Dashboard");

    JSONArray elements = new JSONArray();

    // Helper to add widgets
    for (Widget widget : m_widgets) {
      JSONObject element = new JSONObject();
      element.put("name",  widget.m_name);
      element.put("widget",  toString(widget.m_type));
      element.put("source",  widget.m_source);
      element.put("size_x",  widget.m_sizeX);
      element.put("size_y",  widget.m_sizeY);
      element.put("pos_x",   widget.m_posX);
      element.put("pos_y",   widget.m_posY);
      elements.add(element);
    }

    markup.put("elements", elements);

    return markup.toJSONString();
  }

  enum WidgetType {
    Basic
  }

  public String toString(WidgetType type) {
    switch (type) {
      case Basic:
        return "Basic";
      default:
        return "Basic";
    }
  }

  class Widget {
    private String   m_name   = "Widget";
    private WidgetType m_type   = WidgetType.Basic;
    private String   m_source = "/HawkViewerMarkup";
    private double   m_sizeX = 200.0;
    private double   m_sizeY = 200.0;
    private double   m_posX  = 0.0;
    private double   m_posY  = 0.0;

    public Widget() {

    }

    public Widget(String   name,
            WidgetType type,
            String   source,
            double   sizeX,
            double   sizeY,
            double   posX,
            double   posY
    ) {
      m_name   = name;
      m_type   = type;
      m_source = source;
      m_sizeX = sizeX;
      m_sizeY = sizeY;
      m_posX  = posX;
      m_posY  = posY;
    }

    public Widget withName  (String   name)   {m_name   = name;   return this; }
    public Widget withType  (WidgetType type)   {m_type   = type;   return this; }
    public Widget withSource(String   source) {m_source = source; return this; }
    public Widget withSizeX (double   sizeX)  {m_sizeX  = sizeX;  return this; }
    public Widget withSizeY (double   sizeY)  {m_sizeY  = sizeY;  return this; }
    public Widget withPosX  (double   posX)   {m_posX   = posX;   return this; }
    public Widget withPosY  (double   posY)   {m_posY   = posY;   return this; }
  }

}
