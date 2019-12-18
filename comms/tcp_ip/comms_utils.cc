#include "comms_utils.h"

namespace comms {
// TODO: class
// These functions make sure class get destructive whenever ctrl+c get pressed.
namespace control {

    void got_signal(int)
    {
        quit.store(true);
    }

    void set_gracefully_exit()
    {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = got_signal;
        sigfillset(&sa.sa_mask);
        sigaction(SIGINT, &sa, NULL);
    }

    bool program_exit()
    {
        return quit.load();
    }
}

// Handing ICP data packging
// e.g. server: send(100byte), send(100byte)
//      client: 20byte = recv(), 110byte = recv(), 70byte = recv
// The protocol is simplely a sperator for each message.
namespace package_sync {

    bool send_control_package(int socket)
    {
        int n = send(socket, control_string, sizeof(control_string), 0);
        if (n == -1) {
            std::cout << "send_control_package fail" << std::endl;
            return false;
        }

        return true;
    }

    // Wait for the header
    SyncStatus wait_for_control_package(int socket, char* buf, int& received_data)
    {
        // TODO: doesn't work if TCP decide to break control message into 2 receive.
        //       or mutiple control pkg beem grouped into the same tcp recv
        constexpr int SMALL_BUFFER_SIZE = 1024;
        char small_buf[SMALL_BUFFER_SIZE];

        for (size_t num_try = 0; num_try < 1000; ++num_try) {
            int n = recv(socket, small_buf, SMALL_BUFFER_SIZE, 0);
            if (n == -1) {
                std::cout << "wait_for_control_package recv fail, retry" << std::endl;
                return SyncStatus::failure;
            }
            if (n == 0) {
                std::cout << "wait_for_control_package server disconnected" << std::endl;
                return SyncStatus::failure;
            }

            char* control_string_ptr = strstr(small_buf, control_string);
            if (control_string_ptr == nullptr) {
                continue;
            } else {
                // strip control_string, copy other data to main buffer
                // assume buff is larger than SMALL_BUFFER_SIZE
                const char* mesage_start = control_string_ptr + sizeof(control_string);
                const char* message_end = small_buf + n;
                const int message_size = message_end - mesage_start;
                // assume buf is long enough.
                memcpy(buf,
                    mesage_start,
                    message_size);
                received_data = message_size;

                return SyncStatus::success;
            }
        }
        std::cout << "wait_for_control_packge timeout" << std::endl;
        return SyncStatus::timeout;
    }
}

// https://beej.us/guide/bgnet/html//index.html
bool sendall(int socket, char* buf, int len)
{
    SLIENT_COUT_CURRENT_SCOPE;

    int sent = 0;
    int bytesleft = len;
    int n = -1;

    const int max_tries = 20;
    int num_try = 0;

    while (sent < len) {
        if (num_try++ > max_tries) {
            std::cout << "send all time out" << std::endl;
            return false;
        }

        n = send(socket, buf + sent, bytesleft, 0);
        if (n <= 0) {
            std::cout << "send fail, retry" << std::endl;

            continue;
        }

        sent += n;
        bytesleft -= n;
        PRINT_NAME_VAR(n);
    }
    PRINT_NAME_VAR(len);
    PRINT_NAME_VAR(sent);
    assert(len == sent);

    return len == sent;
}

// https://beej.us/guide/bgnet/html//index.html
bool recv_all(int socket, char* buf, int want_size_byte)
{
    SLIENT_COUT_CURRENT_SCOPE;

    int total = 0;
    int n = -1;
    int want = want_size_byte;

    while (total < want_size_byte) {
        n = recv(socket, buf + total, want - total, 0);
        if (n == -1) {
            std::cout << "recv fail" << std::endl;
            return false;
        }
        if (n == 0) {
            std::cout << "server disconnect" << std::endl;
            return false;
        }
        total += n;
        std::cout << "recv n bytes: " << n << std::endl;
    }

    std::cout << "total :" << total << std::endl;

    assert(want_size_byte == total && "len != total");

    return true;
}

void sigchld_handler(int s)
{
    (void)s; // quiet unused variable warning

    // waitpid() might overwrite errno, so we save and restore it:
    int saved_errno = errno;

    while (waitpid(-1, NULL, WNOHANG) > 0)
        ;

    errno = saved_errno;
}

// get sockaddr, IPv4 or IPv6:
void* get_in_addr(struct sockaddr* sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

bool kill_dead_processes()
{
    struct sigaction sa;
    sa.sa_handler = sigchld_handler; // reap all dead processes
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGCHLD, &sa, NULL) == -1) {
        perror("sigaction");
        return false;
    }

    return true;
}

void print_current_ip()
{
    struct ifaddrs* ifAddrStruct = NULL;
    struct ifaddrs* ifa = NULL;
    void* tmpAddrPtr = NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
        } else if (ifa->ifa_addr->sa_family == AF_INET6) {
        }
    }
    if (ifAddrStruct != NULL)
        freeifaddrs(ifAddrStruct);
    std::cout << "please make sure you connected to the wlp ip" << std::endl;
}

// Thank you hbristow!
Arguments tcp_ip_image_server_argparse(int argc, const char** argv)
{
    ArgumentParser parser;
    parser.addArgument("--ip", 1, false);
    parser.addArgument("--port", 1, false);
    parser.addArgument("--image-dir", 1, false);

    parser.parse(argc, argv);

    Arguments args;
    args.ip = parser.retrieve<std::string>("ip");
    args.port = parser.retrieve<std::string>("port");
    args.image_dir = parser.retrieve<std::string>("image-dir");
    return args;
}

// Thank you hbristow!
Arguments tcp_ip_argparse(int argc, const char** argv)
{
    ArgumentParser parser;
    parser.addArgument("--ip", 1, false);
    parser.addArgument("--port", 1, false);

    parser.parse(argc, argv);

    Arguments args;
    args.ip = parser.retrieve<std::string>("ip");
    args.port = parser.retrieve<std::string>("port");

    return args;
}
}
