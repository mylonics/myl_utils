#pragma once

#include "lora_device.h"

struct registered_client {
  uint32_t mac_id;
  uint8_t node_id;
  size_t missed_replies;
};

class LoraServer : public LoraDevice {
 public:
  LoraServer(const struct device *const lora_dev, uint8_t net_id, lora_msg_cb msg_cb)
      : LoraDevice{lora_dev, net_id, msg_cb, true} {
    registered_ = true;

    k_mutex_init(&my_mutex);
  };

  bool SendMessage(uint8_t dest_id, uint8_t msg_id, uint8_t *msg_data, uint8_t msg_length) {
    k_mutex_lock(&my_mutex, K_FOREVER);
    DECLARE_MYL_UTILS_LOG();
    LOG_INF("Sending Msg %d to dest %d", msg_id, dest_id);
    bool ret = Transmit(dest_id, msg_id, msg_data, msg_length);
    k_mutex_unlock(&my_mutex);
    return ret;
  }

  void TempRunner() {
    rx_message_time = 5000;
    while (true) {
      Receive();
    }
  }

  void Runner() {
    int broadcast_request_count = 0;
    while (true) {
      k_mutex_lock(&my_mutex, K_FOREVER);
      RegisterClients();
      for (size_t i = 0; i < registered_client_length; i++) {
        SendNodeRequestPacket(clients[i].node_id);
      }
      broadcast_request_count++;
      if (broadcast_request_count >= 1) {
        SendNodeRequestPacket(0);
        broadcast_request_count = 0;
      }
      k_mutex_unlock(&my_mutex);
      k_sleep(K_MSEC(1));
    }
  }

 private:
  registered_client clients[20];
  size_t registered_client_length = 0;
  bool transmitting{};
  bool waiting{};
  uint64_t send_message_time{};
  struct k_mutex my_mutex;
  uint32_t client_mac_{};

  void AddClientToRegistrationList(uint32_t client_mac) {
    DECLARE_MYL_UTILS_LOG();
    bool already_registered = false;
    uint8_t client_node_id = 0;

    for (size_t i = 0; i < registered_client_length; i++) {
      if (clients[i].mac_id == client_mac) {
        already_registered = true;
        client_node_id = clients[i].node_id;
        break;
      }
    }

    if (!already_registered) {
      client_node_id = registered_client_length + 1;
      clients[registered_client_length].mac_id = client_mac;
      clients[registered_client_length].node_id = client_node_id;
      registered_client_length++;
      LOG_INF("Registering New Client %d %d", client_mac, client_node_id);
    } else {
      LOG_INF("Re-Registering Client %d %d", client_mac, client_node_id);
    }
    clients[client_node_id - 1].missed_replies = 10;
  }

  void RegisterClients() {
    for (size_t i = 0; i < registered_client_length; i++) {
      if (clients[i].missed_replies >= 10) {
        clients[i].missed_replies = 0;
        uint8_t data[6];

        data[0] = (uint8_t)NETWORK_MSG_IDS::REGISTER_REPLY;
        data[1] = clients[i].node_id;
        data[2] = (uint8_t)((clients[i].mac_id >> 24) & 0xFF);
        data[3] = (uint8_t)((clients[i].mac_id >> 16) & 0xFF);
        data[4] = (uint8_t)((clients[i].mac_id >> 8) & 0xFF);
        data[5] = (uint8_t)((clients[i].mac_id) & 0xFF);
        DECLARE_MYL_UTILS_LOG();
        LOG_INF("Sending Registration to node %d mac %d", clients[i].node_id, clients[i].mac_id);
        Transmit(0, 0, data, 6, true);
      }
    }
  }

  void SendNodeRequestPacket(uint8_t node_id) {
    uint8_t net_msg_id = (uint8_t)NETWORK_MSG_IDS::BROADCAST;
    if (node_id) {
      net_msg_id = (uint8_t)NETWORK_MSG_IDS::NODE_REQUEST;
      clients[node_id - 1].missed_replies++;
    }
    DECLARE_MYL_UTILS_LOG();
    LOG_DBG("Sending Node Request %d ", node_id);
    Transmit(node_id, 0, &net_msg_id, 1);
    Receive();
  }

  void HandleMessage() {
    DECLARE_MYL_UTILS_LOG();
    if (!rx_msg_header.msg_id) {
      switch ((NETWORK_MSG_IDS)rx_msg_header.data[0]) {
        case NETWORK_MSG_IDS::REGISTER:
          client_mac_ = rx_msg_header.data[1] << 24 | rx_msg_header.data[2] << 16 | rx_msg_header.data[3] << 8 |
                        rx_msg_header.data[4];
          AddClientToRegistrationList(client_mac_);
          break;
        case NETWORK_MSG_IDS::NODE_REPLY:
          LOG_DBG("Node %d is replying", rx_net_header.src);
          clients[rx_net_header.src - 1].missed_replies = 0;
          break;
        default:
          break;
      }

      LOG_DBG("got network message\n");
    } else {
      LOG_INF("Node %d sent msg %d ", rx_net_header.src, rx_msg_header.msg_id);
      // log in that client is still active or not
      // rx_net_header.src;
    }
  }
};
