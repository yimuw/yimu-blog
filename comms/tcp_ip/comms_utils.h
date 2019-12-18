#pragma once

// TODO: remove some?
#include <arpa/inet.h>
#include <assert.h>
#include <atomic>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "third_party/argparse/argparse.hpp"

namespace comms {
// C programmers are crazy...
using Socket = int;

#define PRINT_NAME_VAR(x) std::cout << #x << " :" << x << std::endl;

namespace internal {
    struct SilentCoutInternal {
        SilentCoutInternal()
        {
            oldCoutStreamBuf = std::cout.rdbuf();
            std::cout.rdbuf(strCout.rdbuf());
        }

        ~SilentCoutInternal()
        {
            std::cout.rdbuf(oldCoutStreamBuf);
        }

        std::ostringstream strCout;
        std::streambuf* oldCoutStreamBuf = nullptr;
    };
}

// TODO: logger
#define SLIENT_COUT_CURRENT_SCOPE internal::SilentCoutInternal var_name_duplicated;
// #define SLIENT_COUT_CURRENT_SCOPE

template <typename T>
char* cast_to_char_ptr(T* const ptr)
{
    return reinterpret_cast<char*>(ptr);
}

// TODO: class
// These functions make sure class get destructive whenever ctrl+c get pressed.
namespace control {
    std::atomic<bool> quit(false); // signal flag

    void got_signal(int);

    void set_gracefully_exit();

    bool program_exit();
}

// Handing ICP data packging
// e.g. server: send(100byte), send(100byte)
//      client: 20byte = recv(), 110byte = recv(), 70byte = recv
// The protocol is simplely a sperator for each message.
namespace package_sync {
    char control_string[] = "I am just some magic number. like 123456789. \
                         I use it to singal the start of a message.   \
                         make sure you dont type it in your message!  \
                         I am curious how do people handle it in      \
                         pratice? Special character? definitely     \
                         not a super long string like this.";

    bool send_control_package(int socket);

    enum class SyncStatus {
        success,
        failure,
        timeout
    };

    // Wait for the header
    SyncStatus wait_for_control_package(int socket, char* buf, int& received_data);
}

// https://beej.us/guide/bgnet/html//index.html
bool sendall(int socket, char* buf, int len);

// https://beej.us/guide/bgnet/html//index.html
bool recv_all(int socket, char* buf, int want_size_byte);

void sigchld_handler(int s);

// get sockaddr, IPv4 or IPv6:
void* get_in_addr(struct sockaddr* sa);

bool kill_dead_processes();

void print_current_ip();

struct Arguments {
    std::string ip;
    std::string port;
    std::string image_dir;
};

// Thank you hbristow!
Arguments tcp_ip_image_server_argparse(int argc, const char** argv);

// Thank you hbristow!
Arguments tcp_ip_argparse(int argc, const char** argv);
}
