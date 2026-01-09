#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace {

constexpr std::chrono::milliseconds kPublishPeriod{ 50 };
const rclcpp::Duration kDisconnectTimeout{ std::chrono::milliseconds(500) };
constexpr int kBindPort            = 25000;
constexpr std::size_t kAxisCount   = 6;
constexpr std::size_t kButtonCount = 10;
constexpr double kMaxZeroPos       = 0.01;
constexpr double kAnalogDiv        = 2048.0;

class RemoteControler6D {
public:
  static std::optional<RemoteControler6D> from_packet(std::string_view bytes, double frequency) {
    if(bytes.size() != 62) {
      return std::nullopt;
    }

    std::array<std::string_view, 18> parts{};
    std::size_t part_index = 0;
    std::size_t start      = 0;
    while(part_index < parts.size() && start <= bytes.size()) {
      std::size_t pos = bytes.find(':', start);
      if(pos == std::string_view::npos) {
        parts[part_index++] = bytes.substr(start);
        break;
      }
      parts[part_index++] = bytes.substr(start, pos - start);
      start               = pos + 1;
    }

    if(part_index != parts.size() || parts.front() != "$RC" || parts.back() != "#\r") {
      return std::nullopt;
    }

    RemoteControler6D out;
    out.frequency = frequency;
    try {
      out.joystick_1_x   = std::stoi(std::string(parts[1]));
      out.joystick_1_y   = std::stoi(std::string(parts[2]));
      out.joystick_1_z   = std::stoi(std::string(parts[3]));
      out.joystick_1_btn = std::stoi(std::string(parts[4]));
      out.joystick_2_x   = std::stoi(std::string(parts[5]));
      out.joystick_2_y   = std::stoi(std::string(parts[6]));
      out.joystick_2_z   = std::stoi(std::string(parts[7]));
      out.joystick_2_btn = std::stoi(std::string(parts[8]));
      out.button_1       = std::stoi(std::string(parts[9]));
      out.button_2       = std::stoi(std::string(parts[10]));
      out.button_3       = std::stoi(std::string(parts[11]));
      out.button_4       = std::stoi(std::string(parts[12]));
      out.button_5       = std::stoi(std::string(parts[13]));
      out.button_6       = std::stoi(std::string(parts[14]));
      out.button_7       = std::stoi(std::string(parts[15]));
      out.button_8       = std::stoi(std::string(parts[16]));
    } catch(...) {
      return std::nullopt;
    }
    return out;
  }

  double frequency{ 0.0 };
  int joystick_1_x{ 0 };
  int joystick_1_y{ 0 };
  int joystick_1_z{ 0 };
  int joystick_1_btn{ 0 };
  int joystick_2_x{ 0 };
  int joystick_2_y{ 0 };
  int joystick_2_z{ 0 };
  int joystick_2_btn{ 0 };
  int button_1{ 0 };
  int button_2{ 0 };
  int button_3{ 0 };
  int button_4{ 0 };
  int button_5{ 0 };
  int button_6{ 0 };
  int button_7{ 0 };
  int button_8{ 0 };
};

class UdpReceiver {
public:
  explicit UdpReceiver(int port) : port_(port) {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if(socket_fd_ < 0) {
      throw std::runtime_error("Failed to create socket");
    }

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(static_cast<uint16_t>(port_));

    if(::bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(socket_fd_);
      throw std::runtime_error("Failed to bind UDP socket");
    }
  }

  ~UdpReceiver() {
    stop();
  }

  void on_receive(std::function<void(const RemoteControler6D &)> cb) {
    callback_ = std::move(cb);
  }

  void start() {
    running_.store(true);
    recv_thread_ = std::thread([this]() { receive_loop(); });
  }

  void stop() {
    running_.store(false);
    if(socket_fd_ >= 0) {
      ::shutdown(socket_fd_, SHUT_RDWR);
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
    if(recv_thread_.joinable()) {
      recv_thread_.join();
    }
  }

private:
  void receive_loop() {
    constexpr std::size_t kBufferSize = 128;
    std::array<char, kBufferSize> buffer{};
    auto previous = std::chrono::steady_clock::now();

    while(running_.load()) {
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);
      ssize_t received =
      ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&sender), &sender_len);
      if(received <= 0) {
        continue;
      }

      auto now    = std::chrono::steady_clock::now();
      double dt   = std::chrono::duration<double>(now - previous).count();
      previous    = now;
      double freq = dt > 0.0 ? 1.0 / dt : 0.0;

      if(callback_) {
        auto packet = std::string_view(buffer.data(), static_cast<std::size_t>(received));
        if(auto data = RemoteControler6D::from_packet(packet, freq)) {
          callback_.value()(*data);
        }
      }
    }
  }

  int port_{ 0 };
  int socket_fd_{ -1 };
  std::atomic<bool> running_{ false };
  std::thread recv_thread_;
  std::optional<std::function<void(const RemoteControler6D &)>> callback_;
};

double map_value(int raw) {
  const double value = static_cast<double>(raw) / kAnalogDiv;
  if(value < kMaxZeroPos && value > -kMaxZeroPos) {
    return 0.0;
  }
  return value;
}

} // namespace

class RemoteJoystickDriver : public rclcpp::Node {
public:
  RemoteJoystickDriver() : rclcpp::Node("remote_joystick_driver"), receiver_(kBindPort) {
    publisher_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    timer_     = create_wall_timer(kPublishPeriod, std::bind(&RemoteJoystickDriver::on_timer, this));
    receiver_.on_receive([this](const RemoteControler6D &data) { handle_receive(data); });
    last_connection_ = now();
    receiver_.start();
    RCLCPP_INFO(get_logger(), "Remote joystick receiver started on UDP port %d", kBindPort);
  }

  ~RemoteJoystickDriver() override {
    receiver_.stop();
  }

private:
  void handle_receive(const RemoteControler6D &data) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if(!connected_) {
      RCLCPP_INFO(get_logger(), "Pilot Connected");
    }

    connected_       = true;
    zero_sent_       = false;
    last_connection_ = now();

    axes_[0]    = map_value(data.joystick_1_x);
    axes_[1]    = map_value(data.joystick_1_y);
    axes_[2]    = map_value(data.joystick_1_z);
    axes_[3]    = map_value(data.joystick_2_x);
    axes_[4]    = map_value(data.joystick_2_y);
    axes_[5]    = map_value(data.joystick_2_z);
    buttons_[0] = data.joystick_1_btn;
    buttons_[1] = data.joystick_2_btn;
    buttons_[2] = data.button_1;
    buttons_[3] = data.button_2;
    buttons_[4] = data.button_3;
    buttons_[5] = data.button_4;
    buttons_[6] = data.button_5;
    buttons_[7] = data.button_6;
    buttons_[8] = data.button_7;
    buttons_[9] = data.button_8;
  }

  void on_timer() {
    std::array<double, kAxisCount> axes_copy{};
    std::array<int32_t, kButtonCount> buttons_copy{};
    bool connected_copy = false;
    bool publish_zero   = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      const auto elapsed = now() - last_connection_;
      if(connected_ && elapsed > kDisconnectTimeout) {
        RCLCPP_WARN(get_logger(), "Pilot Disconnected");
        connected_   = false;
        publish_zero = !zero_sent_;
        zero_sent_   = true;
      }

      if(!connected_) {
        axes_.fill(0.0);
        buttons_.fill(0);
      }

      axes_copy      = axes_;
      buttons_copy   = buttons_;
      connected_copy = connected_;
    }

    if(connected_copy || publish_zero) {
      sensor_msgs::msg::Joy msg;
      msg.header.frame_id = "joy";
      msg.header.stamp    = get_clock()->now();
      msg.axes.assign(axes_copy.begin(), axes_copy.end());
      msg.buttons.assign(buttons_copy.begin(), buttons_copy.end());
      publisher_->publish(msg);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  UdpReceiver receiver_;
  std::mutex data_mutex_;
  std::array<double, kAxisCount> axes_{};
  std::array<int32_t, kButtonCount> buttons_{};
  rclcpp::Time last_connection_;
  bool connected_{ false };
  bool zero_sent_{ false };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemoteJoystickDriver>());
  rclcpp::shutdown();
  return 0;
}