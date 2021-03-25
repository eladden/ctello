//  CTello is a C++ library to interact with the DJI Ryze Tello Drone
//  Copyright (C) 2020 Carlos Perez-Lopez
//
//  This library is free software: you can redistribute it and/or modify
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

#include "ctello.h"

#include <errno.h>
#include <memory.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include "spdlog/spdlog.h"

const char* const LOG_PATTERN = "[%D %T] [ctello] [%^%l%$] %v";

namespace
{
// Reads the spdlog level from the given environment variable name.
spdlog::level::level_enum GetLogLevelFromEnv(const std::string& var_name)
{
    // clang-format off
    std::unordered_map<std::string, spdlog::level::level_enum> name_to_enum = {
        {"trace", spdlog::level::trace},
        {"debug", spdlog::level::debug},
        {"info", spdlog::level::info},
        {"warn", spdlog::level::warn},
        {"error", spdlog::level::err},
        {"critical", spdlog::level::critical},
        {"off", spdlog::level::off}
    };
    // clang-format on
    const char* const name_c_str = std::getenv(var_name.c_str());
    if (name_c_str == nullptr)
    {
        // Info is the default
        return spdlog::level::info;
    }
    const std::string name{name_c_str};
    return name_to_enum[name];
}

// Binds the given socket file descriptor ot the given port.
// Returns whether it suceeds or not and the error message.
std::pair<bool, std::string> BindSocketToPort(const int sockfd, const int port)
{
    sockaddr_in listen_addr{};
    // htons converts from host byte order to network byte order.
    listen_addr.sin_port = htons(port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    listen_addr.sin_family = AF_INET;
    int result = bind(sockfd, reinterpret_cast<sockaddr*>(&listen_addr),
                      sizeof(listen_addr));

    if (result == -1)
    {
        std::stringstream ss;
        ss << "bind to " << port << ": " << errno;
        ss << " (" << strerror(errno) << ")";
        return {false, ss.str()};
    }

    return {true, ""};
}

// Finds the socket address given an ip and a port.
// Returns whether it suceeds or not and the error message.
std::pair<bool, std::string> FindSocketAddr(const char* const ip,
                                            const char* const port,
                                            sockaddr_storage* const addr)
{
    addrinfo* result_list{nullptr};
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    int result = getaddrinfo(ip, port, &hints, &result_list);

    if (result != 0)
    {
        std::stringstream ss;
        ss << "getaddrinfo: " << result;
        ss << " (" << gai_strerror(result) << ") ";
        return {false, ss.str()};
    }

    memcpy(addr, result_list->ai_addr, result_list->ai_addrlen);
    freeaddrinfo(result_list);

    return {true, ""};
}

// Sends a string of bytes to the given destination address.
// Returns the number of sent bytes and, if -1, the error message.
std::pair<int, std::string> SendTo(const int sockfd,
                                   sockaddr_storage& dest_addr,
                                   const std::vector<unsigned char>& message)
{
    const socklen_t addr_len{sizeof(dest_addr)};
    int result = sendto(sockfd, message.data(), message.size(), 0,
                        reinterpret_cast<sockaddr*>(&dest_addr), addr_len);
    std::stringstream ss;
    if (result == -1)
    {
        ss << "sendto: " << errno;
        ss << " (" << strerror(errno) << ")";
        return {-1, ss.str()};
    }
    return {result, ""};
}

// Receives a text response from the given destination address.
// Returns the number of received bytes and, if -1, the error message.
std::pair<int, std::string> ReceiveFrom(const int sockfd,
                                        sockaddr_storage& addr,
                                        std::vector<unsigned char>& buffer,
                                        const int buffer_size = 1024,
                                        const int flags = MSG_DONTWAIT)
{
    socklen_t addr_len{sizeof(addr)};
    buffer.resize(buffer_size, '\0');
    // MSG_DONTWAIT -> Non-blocking
    // recvfrom is storing (re-populating) the sender address in addr.
    int result = recvfrom(sockfd, buffer.data(), buffer_size, flags,
                          reinterpret_cast<sockaddr*>(&addr), &addr_len);
    if (result == -1)
    {
        std::stringstream ss;
        ss << "recvfrom: " << errno;
        ss << " (" << strerror(errno) << ")";
        return {-1, ss.str()};
    }

    return {result, ""};
}
}  // namespace

namespace ctello
{
Tello::Tello()
{
    createSockets();
    spdlog::set_pattern(LOG_PATTERN);
    auto log_level = ::GetLogLevelFromEnv("SPDLOG_LEVEL");
    spdlog::set_level(log_level);
}

Tello::~Tello()
{
    closeSockets();
    if (!logFileName.empty()){
        auto now = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());
        std::string date = std::ctime(&now);
        telloLogFile << "end time:" << date << std::endl;
        telloLogFile.close();
    }
}
void Tello::closeSockets()
{
    close(m_command_sockfd);
    close(m_state_sockfd);
}
void Tello::createSockets()
{
    m_command_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    m_state_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
}
bool Tello::Bind(const int local_client_command_port)
{
    // UDP Client to send commands and receive responses
    auto result =
        ::BindSocketToPort(m_command_sockfd, local_client_command_port);
    if (!result.first)
    {
        spdlog::error(result.second);
        return false;
    }
    m_local_client_command_port = local_client_command_port;
    result = ::FindSocketAddr(TELLO_SERVER_IP, TELLO_SERVER_COMMAND_PORT,
                              &m_tello_server_command_addr);
    if (!result.first)
    {
        spdlog::error(result.second);
        return false;
    }

    // Local UDP Server to listen for the Tello Status
    result = ::BindSocketToPort(m_state_sockfd, LOCAL_SERVER_STATE_PORT);
    if (!result.first)
    {
        spdlog::error(result.second);
        return false;
    }

    // Finding Tello
    spdlog::info("Finding Tello ...");
    FindTello();
    spdlog::info("Entered SDK mode");

    ShowTelloInfo();
    struct stat info;

    if (stat("/home", &info) == 0 && (info.st_mode & S_IFDIR))
    {
        char userName[255];
        getlogin_r(userName, 255);
        std::string telloFolder = "/home/" + std::string(userName) + "/tello_logs";
        if (!(stat(telloFolder.c_str(), &info) == 0 && info.st_mode & S_IFDIR))
        {
            system(("mkdir " + telloFolder).c_str());
        }

        auto now = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());
        std::string date = std::ctime(&now);
        logFileName = telloFolder + "/" + date + ".log";
        telloLogFile = std::ofstream(logFileName);
        telloLogFile << "start time:" << date << std::endl;
    }
    return true;
}

void Tello::FindTello()
{
    do
    {
        SendCommand("command");
        sleep(1);
    } while (!(ReceiveResponse()));
}
std::string Tello::GetTelloName()
{
    std::experimental::optional<std::string> response;
    SendCommand("sn?");
    while (!(response = ReceiveResponse()))
        ;
    return response.value();
}
bool Tello::EasyLanding(){
    while (!SendCommandWithResponse("down 20"));
    SendCommand("land");
}
int Tello::GetBatteryStatus()
{
    std::experimental::optional<std::string> response;
    SendCommand("battery?");
    while (!(response = ReceiveResponse()))
        ;
    if (response)
    {
        return std::stoi(response.value());
    }
    return 0;
}
void Tello::ShowTelloInfo()
{
    std::experimental::optional<std::string> response;

    SendCommand("sn?");
    while (!(response = ReceiveResponse()))
        ;
    spdlog::info("Serial Number: {0}", *response);

    SendCommand("sdk?");
    while (!(response = ReceiveResponse()))
        ;
    spdlog::info("Tello SDK:     {0}", *response);

    SendCommand("wifi?");
    while (!(response = ReceiveResponse()))
        ;
    spdlog::info("Wi-Fi Signal:  {0}", *response);

    SendCommand("battery?");
    while (!(response = ReceiveResponse()))
        ;
    spdlog::info("Battery:       {0}", *response);
}
bool Tello::SendCommand(const std::string& command)
{
    const std::vector<unsigned char> message{command.begin(), command.end()};
    const auto result =
        ::SendTo(m_command_sockfd, m_tello_server_command_addr, message);
    const int bytes{result.first};
    if (bytes == -1)
    {
        spdlog::error(result.second);
        return false;
    }
    spdlog::debug("127.0.0.1:{} >>>> {} bytes >>>> {}:{}: {}",
                  m_local_client_command_port, bytes, TELLO_SERVER_IP,
                  TELLO_SERVER_COMMAND_PORT, command);
    return true;
}
bool Tello::SendCommandWithResponse(const std::string& command)
{
    const std::vector<unsigned char> message{command.begin(), command.end()};
    auto start = std::chrono::system_clock::now();
    if (!logFileName.empty()){
        auto now = std::chrono::system_clock::to_time_t(start);
        std::string date = std::ctime(&now);
        telloLogFile << "time:" << date << ",battery:" << GetBatteryStatus()
                     << ",command:" << command << ",";
    }


    const auto result =
        ::SendTo(m_command_sockfd, m_tello_server_command_addr, message);
    const int bytes{result.first};
    if (bytes == -1)
    {
        if (!logFileName.empty()){
            telloLogFile << "failed" << std::endl;
        }
        spdlog::error(result.second);
        return false;
    }
    char answer[2048] = {0};
    while (!answer[0])
    {
        recv(m_command_sockfd, answer, 2048, 0);
    }
    std::string strAnswer(answer);
    spdlog::info(strAnswer);
    spdlog::debug("127.0.0.1:{} >>>> {} bytes >>>> {}:{}: {}",
                  m_local_client_command_port, bytes, TELLO_SERVER_IP,
                  TELLO_SERVER_COMMAND_PORT, command);

    // Some computation here
    bool isSuccess = strAnswer.find("OK") != std::string::npos ||
                     strAnswer.find("ok") != std::string::npos;
    if (!logFileName.empty()){
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        telloLogFile << "completed in:" << elapsed_seconds.count()
                     << ",battery:" << GetBatteryStatus()
                     << ",success:" << isSuccess << std::endl;
    }

    return isSuccess;
}

std::experimental::optional<std::string> Tello::ReceiveResponse()
{
    const int size{32};
    std::vector<unsigned char> buffer(size, '\0');
    const auto result = ::ReceiveFrom(
        m_command_sockfd, m_tello_server_command_addr, buffer, size);
    const int bytes{result.first};
    if (bytes < 1)
    {
        return {};
    }
    std::string response{buffer.begin(), buffer.begin() + bytes};
    // Some responses contain trailing white spaces.
    response.erase(response.find_last_not_of(" \n\r\t") + 1);
    spdlog::debug("127.0.0.1:{} <<<< {} bytes <<<< {}:{}: {}",
                  m_local_client_command_port, bytes, TELLO_SERVER_IP,
                  TELLO_SERVER_COMMAND_PORT, response);
    return response;
}

std::experimental::optional<std::string> Tello::GetState()
{
    sockaddr_storage addr;
    const int size{1024};
    std::vector<unsigned char> buffer(size, '\0');
    const auto result = ::ReceiveFrom(m_state_sockfd, addr, buffer, size);
    const int bytes{result.first};
    if (bytes < 1)
    {
        return {};
    }
    std::string response{buffer.begin(), buffer.end()};
    // Some responses contain trailing white spaces.
    response.erase(response.find_last_not_of(" \n\r\t") + 1);
    spdlog::debug("127.0.0.1:{} <<<< {} bytes <<<< {}:{}: <state>",
                  m_local_client_command_port, bytes, TELLO_SERVER_IP,
                  TELLO_SERVER_COMMAND_PORT);
    return response;
}
}  // namespace ctello
