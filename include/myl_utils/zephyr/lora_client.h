#pragma once

#include "lora_device.h"

class LoraClient : public LoraDevice {
 public:
  LoraClient(const struct device *const lora_dev, uint8_t net_id, uint32_t mac_id, lora_msg_cb msg_cb)
      : LoraDevice{lora_dev, net_id, msg_cb, false}, mac_id_{mac_id} {};

  void queue_message(uint8_t dest_id, uint8_t msg_id, uint8_t *msg_data, uint16_t msg_length) {
    if (msg_length < max_queue_msg_size_) {
      queued_msg_header_.msg_id = msg_id;
      queued_msg_header_.length = msg_length;
      memcpy(queued_buffer_, msg_data, msg_length);
      queued_destination_ = dest_id;
      msg_queued_ = true;
    }
  }

  void register_onto_network() {
    uint8_t data[5];
    data[0] = (uint8_t)NETWORK_MSG_IDS::REGISTER;
    data[1] = mac_id_ >> 24;
    data[2] = mac_id_ >> 16;
    data[3] = mac_id_ >> 8;
    data[4] = mac_id_;

    LOG_INF("Sending Registration request");
    transmit(0, 0, data, 5);
  }

  void Runner() {
    while (true) {
      receive();
      if (registration_request) {
        register_onto_network();
        registration_request = false;
      } else if (reply_request) {
        if (msg_queued_) {
          transmit(queued_destination_, queued_msg_header_.msg_id, queued_buffer_, queued_msg_header_.length);
          msg_queued_ = false;
          reply_request = false;
        } else {
          uint8_t data = (uint8_t)NETWORK_MSG_IDS::NODE_REPLY;
          LOG_INF("Sending NODE reply");
          transmit(0, 0, &data, 1);
          reply_request = false;
        }
      }
    }
  }

 private:
  uint32_t mac_id_;
  uint64_t last_sync_time_;

  static const size_t max_queue_msg_size_ = 1024;
  uint8_t queued_destination_;
  message_header queued_msg_header_;
  uint8_t queued_buffer_[max_queue_msg_size_];
  bool msg_queued_{};
  size_t no_request_count{};
  bool registration_request{};
  bool reply_request{};

  void handle_message() {
    uint32_t mac_id = 0;
    if (rx_msg_header.msg_id == 0) {
      LOG_INF("got network message %d\n", rx_msg_header.data[0]);
      switch ((NETWORK_MSG_IDS)rx_msg_header.data[0]) {
        case NETWORK_MSG_IDS::BROADCAST:
          if (!registered_) {
            registration_request = true;
          } else {
            no_request_count++;
            if (no_request_count > 5) {
              registered_ = false;
              registration_request = true;
              no_request_count = 0;
            }
          }
          break;

        case NETWORK_MSG_IDS::REGISTER_REPLY:
          mac_id = rx_msg_header.data[2] << 24 | rx_msg_header.data[3] << 16 | rx_msg_header.data[4] << 8 |
                   rx_msg_header.data[5];
          if (mac_id == mac_id_) {
            node_id_ = rx_msg_header.data[1];
            registered_ = true;
          }
          break;

        case NETWORK_MSG_IDS::NODE_REQUEST:
          if (!registration_request) {
            if (rx_net_header.dest == node_id_) {
              no_request_count = 0;
              reply_request = true;
            }
          }
          break;

        case NETWORK_MSG_IDS::REGISTER:
        default:
          break;
      }
    }
  }
};
