#pragma once

#include <optional>

#include <frc/XboxController.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/RobotController.h> 

#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>

#include "ConstantsRoboRio.h"

#define ButtonFunction(button)\
ButtonConfiguration *button(                                                          \
    ButtonMode mode,                                                                  \
    frc2::CommandPtr command,                                                         \
    std::optional<frc2::CommandPtr> command2 = std::nullopt)                          \
{                                                                                     \
    frc2::JoystickButton btn(m_controller, Buttons::button);                          \
    if (mode == ButtonMode::DoubleTap)                                                \
    {                                                                                 \
        btn.Debounce(50_ms)                                                           \
            .OnTrue(std::move(command))                                               \
            .MultiPress(2, 0.5_s)                                                     \
            .OnTrue(std::move(command2.value_or(frc2::InstantCommand{}.ToPtr())));    \
    }                                                                                 \
    else                                                                              \
    {                                                                                 \
        btn.OnTrue(std::move(command))                                                \
              .OnFalse(std::move(command2.value_or(frc2::InstantCommand{}.ToPtr()))); \
    }                                                                                 \
                                                                                      \
    return this;                                                                      \
}                                                                                     \

enum Buttons
{
    A                =   1,
    B                =   2,
    X                =   3,
    Y                =   4,
    LeftBumper       =   5,
    RightBumper      =   6,
    Back             =   7,
    Start            =   8,
    LeftStickButton  =   9,
    RightStickButton =  10,
    Pov_0            =   0,
    Pov_45           =  45,
    Pov_90           =  90,
    Pov_135          = 135,
    Pov_180          = 180,
    Pov_225          = 225,
    Pov_270          = 270,
    Pov_315          = 315
};

enum ButtonMode
{
    DoubleTap,
    Hold
};

class Controller : frc2::SubsystemBase
{
    private:
        class ButtonConfiguration
        {
            public:
                ButtonConfiguration(frc::XboxController *controller) 
                    : m_controller{controller}
                {

                }

                ButtonFunction(A)
                ButtonFunction(B)
                ButtonFunction(X)
                ButtonFunction(Y)
                ButtonFunction(LeftBumper)
                ButtonFunction(RightBumper)
                ButtonFunction(Back)
                ButtonFunction(Start)
                ButtonFunction(LeftStickButton)
                ButtonFunction(RightStickButton)
                ButtonFunction(Pov_0)
                ButtonFunction(Pov_45)
                ButtonFunction(Pov_90)
                ButtonFunction(Pov_135)
                ButtonFunction(Pov_180)
                ButtonFunction(Pov_225)
                ButtonFunction(Pov_270)
                ButtonFunction(Pov_315)

            private:
                frc::XboxController *m_controller;
        };

    public:
        Controller(UsbPort_t port)
            : m_configs{&frc::XboxController{port}}
        {

        }

        ButtonConfiguration *ConfigureButtons()
        {
            return &m_configs;
        }

    private:
        ButtonConfiguration m_configs;
};

class NoFrc2Controller
{
    private:
        class ButtonConfiguration
        {
            public:
                ButtonConfiguration(frc::XboxController *controller) 
                    : m_controller{controller}
                {

                }

                ButtonConfiguration *A(     
                    ButtonMode mode,     
                    frc2::CommandPtr command,     
                    std::optional<frc2::CommandPtr> optionalCommand2 = std::nullopt)     
                {
                    auto command2 = optionalCommand2.value_or(frc2::InstantCommand{}.ToPtr());


                    m_triggers.push([&]() {
                        // Check if the button is pressed and
                        // convert the POVs to real array indexes
                        int index = Buttons::A;
                        bool isPressed = false;
                        if (index > Buttons::RightStickButton || index == Buttons::Pov_0)
                        {
                            isPressed = m_controller->GetPov(index);

                            if (index == Buttons::Pov_0)   index =  0;
                            if (index == Buttons::Pov_45)  index = 11;
                            if (index == Buttons::Pov_90)  index = 12;
                            if (index == Buttons::Pov_135) index = 13;
                            if (index == Buttons::Pov_180) index = 14;
                            if (index == Buttons::Pov_225) index = 15;
                            if (index == Buttons::Pov_270) index = 16;
                            if (index == Buttons::Pov_315) index = 17;
                        } 
                        else
                        {
                            isPressed = m_controller->GetRawButton(index);
                        }

                        if (mode == ButtonMode::DoubleTap)     
                        {

                            if (m_controller->GetAButton() && 
                                m_lastPressed[index] - (frc::RobotController::GetTime() * 1_us).convert<units::second_t>() < 0.5_s)
                            {
                                frc2::CommandScheduler::GetInstance().Schedule(command2);
                            } 
                            else if (m_controller->GetAButton)
                                
                            m_lastPressed[index] = (frc::RobotController::GetTime() * 1_us).convert<units::second_t>();
                        }
                        else  
                        {     
                            if (m_controller->GetAButton())
                            {
                                frc2::CommandScheduler::GetInstance().Schedule(command);
                            } else
                            {
                                frc2::CommandScheduler::GetInstance().Schedule(command2);
                            }
                        }
                    });
                    return this;
                }

            private:

                frc::XboxController *m_controller;
                std::array<std::function<void()>, 18> m_triggers;
                std::array<units::second_t, 18>       m_lastPressed;
        };
    
    public:
        
        
    private:

        frc::XboxController m_controller;
};