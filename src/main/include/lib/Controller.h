// #pragma once

// #include <optional>

// #include <frc/MathUtil.h>

// #include <frc/XboxController.h>

// #include <frc2/command/SubsystemBase.h>
// #include <frc/RobotController.h> 
// #include <frc/DriverStation.h>

// #include <frc2/command/Commands.h>
// #include <frc2/command/CommandPtr.h>
// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/WaitCommand.h>
// #include <frc2/command/button/JoystickButton.h>
// #include <frc2/command/button/POVButton.h>

// #include "ConstantsRoboRio.h"
// #include "lib/Logging.h"

// enum Buttons
// {
//     A                =   1,
//     B                =   2,
//     X                =   3,
//     Y                =   4,
//     LeftBumper       =   5,
//     RightBumper      =   6,
//     Back             =   7,
//     Start            =   8,
//     LeftStickButton  =   9,
//     RightStickButton =  10,
//     Pov_0            =   0,
//     Pov_45           =  45,
//     Pov_90           =  90,
//     Pov_135          = 135,
//     Pov_180          = 180,
//     Pov_225          = 225,
//     Pov_270          = 270,
//     Pov_315          = 315
// };

// enum ButtonMode
// {
//     DoubleTap,
//     Hold
// };

// #define CUSTOM_BUTTON_FUNCTION(button)                                                                       \
// ButtonConfiguration *button(                                                                                 \
//     ButtonMode mode,                                                                                         \
//     frc2::CommandPtr command,                                                                                \
//     frc2::CommandPtr command2 = frc2::cmd::None())                                                           \
// {                                                                                                            \
//     int index = Buttons::button;                                                                             \
//     if (index > 10)                                                                                          \
//     {                                                                                                        \
//         index = 10 + (index / 45);                                                                           \
//     }                                                                                                        \
//     m_triggers[index] = [&]() {                                                                              \
//         /*Check if the button is pressed and*/                                                               \
//         /*convert the POVs to real array indexes*/                                                           \
//         int index = Buttons::button;                                                                         \
//         bool isPressed = false;                                                                              \
//         if (index > 10)                                                                                      \
//         {                                                                                                    \
//             isPressed = m_controller->GetPOV(index);                                                         \
//             index = 10 + (index / 45);                                                                       \
//         }                                                                                                    \
//         else                                                                                                 \
//         {                                                                                                    \
//             isPressed = m_controller->GetRawButton(index);                                                   \
//         }                                                                                                    \
//         if (mode == ButtonMode::DoubleTap)                                                                   \
//         {                                                                                                    \
//             if (isPressed && (frc::DriverStation::GetMatchTime() - m_lastPressed[index]) <= m_doubleTapTime) \
//             {                                                                                                \
//                 frc2::CommandScheduler::GetInstance().Schedule(command2);                                    \
//             }                                                                                                \
//             else if (isPressed)                                                                              \
//             {                                                                                                \
//                 frc2::CommandScheduler::GetInstance().Schedule(command);                                     \
//             }                                                                                                \
//             m_lastPressed[index] = frc::DriverStation::GetMatchTime();                                       \
//         }                                                                                                    \
//         else if (mode == ButtonMode::Hold)                                                                   \
//         {                                                                                                    \
//             if (isPressed)                                                                                   \
//             {                                                                                                \
//                 frc2::CommandScheduler::GetInstance().Schedule(command);                                     \
//             }                                                                                                \
//             else                                                                                             \
//             {                                                                                                \
//                 frc2::CommandScheduler::GetInstance().Schedule(command2);                                    \
//             }                                                                                                \
//         }                                                                                                    \
//     };                                                                                                       \
//     return this;                                                                                             \
// }                                                                                                            \

// class Controller
// {
//     private:

//         class ButtonConfiguration
//         {
//             public:
//                 struct JoystickAxis
//                 {
//                     double x;
//                     double y;
//                 };

//                 ButtonConfiguration(frc::XboxController *controller) : 
//                     m_controller{controller}
//                 {
//                     // Set all to 0 to avoid undefined behavior
//                     for (auto &lastPress : m_lastPressed)
//                         lastPress = 0.0_s;
//                 }

//                 CUSTOM_BUTTON_FUNCTION(A)
//                 CUSTOM_BUTTON_FUNCTION(B)
//                 CUSTOM_BUTTON_FUNCTION(X)
//                 CUSTOM_BUTTON_FUNCTION(Y)
//                 CUSTOM_BUTTON_FUNCTION(LeftBumper)
//                 CUSTOM_BUTTON_FUNCTION(RightBumper)
//                 CUSTOM_BUTTON_FUNCTION(Back)
//                 CUSTOM_BUTTON_FUNCTION(Start)
//                 CUSTOM_BUTTON_FUNCTION(LeftStickButton)
//                 CUSTOM_BUTTON_FUNCTION(RightStickButton)
//                 CUSTOM_BUTTON_FUNCTION(Pov_0)
//                 CUSTOM_BUTTON_FUNCTION(Pov_45)
//                 CUSTOM_BUTTON_FUNCTION(Pov_90)
//                 CUSTOM_BUTTON_FUNCTION(Pov_135)
//                 CUSTOM_BUTTON_FUNCTION(Pov_180)
//                 CUSTOM_BUTTON_FUNCTION(Pov_225)
//                 CUSTOM_BUTTON_FUNCTION(Pov_270)
//                 CUSTOM_BUTTON_FUNCTION(Pov_315)

//                 ButtonConfiguration *ConfigureAxis(std::function<frc2::CommandPtr(JoystickAxis, JoystickAxis)> handler)
//                 {
//                     m_axisHandler = handler;
//                     m_triggers[18] = [&]()
//                     {
//                         frc2::CommandScheduler::GetInstance().Schedule(m_axisHandler(
//                             JoystickAxis{frc::ApplyDeadband(m_controller->GetLeftX(),  m_deadZone) * (m_isLXReversed ? -1.0 : 1.0),
//                                          frc::ApplyDeadband(m_controller->GetLeftY(),  m_deadZone) * (m_isLYReversed ? -1.0 : 1.0)},
//                             JoystickAxis{frc::ApplyDeadband(m_controller->GetRightX(), m_deadZone) * (m_isRXReversed ? -1.0 : 1.0),
//                                          frc::ApplyDeadband(m_controller->GetRightY(), m_deadZone) * (m_isRYReversed ? -1.0 : 1.0)})
//                         );
//                     };

//                     return this;
//                 }

//                 ButtonConfiguration *ReverseLX() { m_isLXReversed = true; return this; }
//                 ButtonConfiguration *ReverseLY() { m_isLYReversed = true; return this; }
//                 ButtonConfiguration *ReverseRX() { m_isRXReversed = true; return this; }
//                 ButtonConfiguration *ReverseRY() { m_isRYReversed = true; return this; }

//                 ButtonConfiguration *SetDoubleTapTime(units::second_t time) { m_doubleTapTime = time; }
//                 ButtonConfiguration *SetDeadZone     (double zone)          { m_deadZone      = zone; }

//                 void Poll()
//                 {
//                     for (auto &trigger : m_triggers)
//                         trigger();
//                 }

//             private:

//                 frc::XboxController                                        *m_controller;
//                 std::array<std::function<void()>, 19>                       m_triggers;
//                 std::array<units::second_t,       18>                       m_lastPressed;
//                 std::function<frc2::CommandPtr(JoystickAxis, JoystickAxis)> m_axisHandler;

//                 units::second_t m_doubleTapTime = 0.5_s;
//                 double          m_deadZone      = 0.06;

//                 bool m_isLXReversed, m_isLYReversed, m_isRXReversed, m_isRYReversed = false;
//         };
    
//     public:
//         Controller(UsbPort_t port) :
//             m_controller{int(port)},
//             m_controlLayers{ButtonConfiguration(&m_controller)}
//         {

//         }

//         ButtonConfiguration *ConfigureLayer(int slot = 0)
//         {
//             if ((m_controlLayers.size() - 1) < slot)
//             {
//                 m_controlLayers[slot] = ButtonConfiguration(&m_controller);
//             }

//             return &m_controlLayers[slot];
//         }

//         // This is not best practice, you should not have access to a controller outside of this class.
//         // This only exists so that its more extensible outside intended use. 
//         // ButtonConfiguration *ConfigureLayer(int slot, ButtonConfiguration config)
//         // {
//         //     m_controlLayers[slot] = config;

//         //     return &m_controlLayers[slot];
//         // }

//         void SetActiveSlot(int slot)
//         {
//             if ((m_controlLayers.size() - 1) < slot)
//             {
//                 m_controlLayers[slot] = ButtonConfiguration(&m_controller);
//                 Log("Controller Error " + m_controller.GetPort(), "Using empty slot!");
//             }
//             m_activeLayer = slot;
//         }

//         void Poll()
//         {
//             m_controlLayers[m_activeLayer].Poll();
//         }
        
//     private:

//         frc::XboxController m_controller;

//         std::vector<ButtonConfiguration> m_controlLayers;
//         int                              m_activeLayer = 0;
// };