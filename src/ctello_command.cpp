//  CTello is a C++ library to interact with the DJI Ryze Tello Drone
//  Copyright (C) 2020 Carlos Perez-Lopez
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>
//
//  You can contact the author via carlospzlz@gmail.com

#include <iostream>

#include "ctello.h"

using ctello::Tello;

const char* const PROMPT = "ctello> ";
// clang-format off
const char* const HELP =
"The Tello SDK includes three basic command types.\n"
"\n"
"  Control Commands (xxx)\n"
"   - Returns “ok” if the command was successful.\n"
"   - Returns “error” or an informational result code if the command failed.\n"
"\n"
"  Set Command (xxx a) to set new sub-parameter values\n"
"   - Returns “ok” if the command was successful.\n"
"   - Returns “error” or an informational result code if the command failed.\n"
"\n"
"  Read Commands (xxx?)\n"
"   - Returns the current value of the sub-parameters.\n"
"\n"
"CONTROL COMMANDS\n"
"  command       Enter SDK mode                                   ok / error\n"
"  takeoff       Auto takeoff                                     ok / error\n"
"  land          Auto landing                                     ok / error\n"
"  streamon      Enable video stream                              ok / error\n"
"  streamoff     Disable video stream                             ok / error\n"
"  emergency     Stop motors immediately                          ok / error\n"
"  up x          Ascend to 'x' cm [20-500]                        ok / error\n"
"  down x        Descend to 'x' cm [20-500]                       ok / error\n"
"  left x        Fly left for 'x' cm [20-500]                     ok / error\n"
"  right x       Fly right for 'x' cm [20-500]                    ok / error\n"
"  forward x     Fly forward for 'x' cm [20-500]                  ok / error\n"
"  back x        Fly backward for 'x' cm [20-500]                 ok / error\n"
"  cw            Rotate 'x' degrees in clockwise [1-360]          ok / error\n"
"  ccw           Rotate 'x' degrees in counterclockwise [1-360]   ok / error\n"
"  flip x        Flip in 'x' direction [l, r, f, b]               ok / error\n"
"  go x y z s    Fly to (x, y, z) at speed 's' [10-100]           ok / error\n"
"  stop          Hovers in the air                                ok / error\n"
"  curve x1 y1 z1 x2 y2 z2 s  Fly describing a curve at speed 's' ok / error\n"
"\n"
"SET COMMANDS\n"
"  speed x       Set speed to 'x' cm/s [10-100]                   ok / error\n"
"  rc a b c      Set remote controller control via four channels  ok / error\n"
"  wifi s p      Set Wi-Fi 's' (ssid) to password 'p'             ok / error\n"
"  mon           Enable mission pad detection                     ok / error\n"
"  moff          Disable mission pad detection                    ok / error\n"
"  mdirection x  Set mission direction detection [0, 1, 2]        ok / error\n"
"  ap s p        Set the Tello to station mode                    ok / error\n"
"\n"
"READ COMMANDS\n"
"  speed?        Obtain current speed (cm/s)                      10 - 100  \n"
"  battery?      Obtain current battery percentage                0 - 100   \n"
"  time?         Obtain current flight time                       time      \n"
"  wifi?         Obtain Wi-Fi SNR                                 snr       \n"
"  sdk?          Obtain the Tello SDK version                     version   \n"
"  sn?           Obtain the Tello serial number                   serial    \n"
"\n";
// clang-format on

int main(const int argc, const char* const args[])
{
    Tello tello{};
    bool bound{false};
    if (argc > 1)
    {
        bound = tello.Bind(atoi(args[1]));
    }
    else
    {
        bound = tello.Bind();
    }
    if (!bound)
    {
        return 0;
    }

    std::string command{""};
    std::cout << PROMPT << std::flush;
    while (std::getline(std::cin, command))
    {
        if (command == "exit")
        {
            return 0;
        }
        if (command == "help")
        {
            std::cout << HELP << std::endl;
        }
        else if (command.size() > 0)
        {
            tello.SendCommand(command);
            // Wait for response
            std::optional<std::string> response;
            do
            {
                response = tello.ReceiveResponse();
            } while (!response);
            std::cout << *response << std::endl;
        }
        std::cout << PROMPT << std::flush;
    }
    return 0;
}
