#pragma once

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace can_comm
{

inline int openCanSocket(
  const std::string & interface, const uint32_t * filter_id = nullptr)
{
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw std::runtime_error("socket: " + std::string(std::strerror(errno)));
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("ioctl(SIOCGIFINDEX) " + interface + ": " + error);
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("bind " + interface + ": " + error);
  }

  if (filter_id != nullptr) {
    struct can_filter filter {};
    filter.can_id = *filter_id;
    filter.can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
      const std::string error = std::strerror(errno);
      ::close(fd);
      throw std::runtime_error("setsockopt(CAN_RAW_FILTER): " + error);
    }
  }

  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("fcntl " + interface + ": " + error);
  }

  return fd;
}

}  // namespace can_comm
