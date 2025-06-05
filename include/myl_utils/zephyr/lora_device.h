#pragma once

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "stdint.h"

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DEFAULT_RADIO_NODE), "No default LoRa radio specified in DT");

#define MAX_DATA_LEN 255
#define MAX_DATA_LENGTH 200

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lora_transceive);

struct network_header {
  uint8_t network;
  uint8_t src;
  uint8_t dest;
};

struct message_header {
  uint8_t msg_id;
  uint8_t length;
  uint8_t *data;
  uint8_t checksum_passed;
};

#define MESSAGE_TIME_MS 1000

typedef void (*lora_msg_cb)(const struct network_header, const struct message_header);

class LoraDevice {
 public:
  enum class NETWORK_MSG_IDS : uint8_t { BROADCAST, REGISTER, REGISTER_REPLY, NODE_REQUEST, NODE_REPLY };
  LoraDevice(const struct device *const lora_dev, uint8_t net_id, lora_msg_cb msg_cb)
      : lora_dev_{lora_dev}, net_id_{net_id}, msg_cb_{msg_cb} {
    if (!device_is_ready(lora_dev)) {
      LOG_ERR("%s Device not ready", lora_dev->name);
      return;
    }
    config.frequency = 915100000;
    config.bandwidth = BW_500_KHZ;
    config.datarate = SF_6;
    config.preamble_len = 8;
    config.coding_rate = CR_4_5;
    config.iq_inverted = false;
    config.public_network = false;
    config.tx_power = 14;
    config.tx = true;

    enable_rx();
  }

 protected:
  network_header rx_net_header;
  message_header rx_msg_header;
  uint8_t node_id_{};
  bool registered_{};

  bool lora_transmit(uint8_t dest_id, uint8_t msg_id, uint8_t *msg_data, uint8_t msg_length) {
    if (msg_id && !registered_) {
      LOG_INF("Can't transmit non-network messages before registration");
      return false;
    }

    output_buffer[0] = net_id_;
    output_buffer[1] = node_id_;
    output_buffer[2] = dest_id;
    output_buffer[3] = msg_id;

    uint16_t checksum = fletcher_checksum_calculation(msg_data, msg_length);

    uint8_t total_parts = (msg_length / MAX_DATA_LENGTH) + 1;

    enable_tx();
    for (int i = 0; i < total_parts; i++) {
      output_buffer[4] = (i + 1) << 4 | total_parts;

      uint8_t data_size = (total_parts - i) == 1 ? msg_length % MAX_DATA_LENGTH : MAX_DATA_LENGTH;

      output_buffer[5] = data_size;
      memcpy(output_buffer + 6, msg_data + (i * MAX_DATA_LENGTH), data_size);

      // need to add checksum
      if ((total_parts - i) == 1) {
        output_buffer[5] = output_buffer[5] + 2;
        output_buffer[6 + data_size] = checksum;
        output_buffer[6 + data_size + 1] = checksum >> 8;
      }
      int ret = lora_send(lora_dev_, output_buffer, 6 + output_buffer[5]);
      if (ret < 0) {
        LOG_ERR("LoRa Send failed %d msg id %d first byte %d", ret, msg_id, msg_data[0]);
        enable_rx();
        return false;
      }
    }
    enable_rx();
    return true;
  };

 private:
  const struct device *const lora_dev_;
  uint8_t net_id_;
  uint8_t mac_id_;
  lora_msg_cb msg_cb_;

  bool reply = false;

  uint8_t output_buffer[256];
  uint8_t input_buffer[2048];
  bool multipart = false;
  uint8_t total_parts;
  bool transmitting_;

  struct lora_modem_config config;

  virtual void handle_message() = 0;

  static void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size, int16_t rssi, int8_t snr,
                              void *user_data) {
    ARG_UNUSED(dev);
    ARG_UNUSED(size);

    LoraDevice *device = (LoraDevice *)user_data;
    ARG_UNUSED(user_data);

    uint8_t current_part = data[4] >> 4;
    if (device->multipart) {
      if (device->rx_net_header.network == data[0] && device->rx_net_header.dest == data[2] &&
          device->rx_net_header.src == data[1]) {
        if (device->rx_msg_header.msg_id != data[3]) {
          device->multipart = false;
          return;
        }
      }
    } else {
      device->rx_msg_header.length = 0;
      device->rx_net_header.network = data[0];
      device->rx_net_header.src = data[1];
      device->rx_net_header.dest = data[2];

      device->rx_msg_header.msg_id = data[3];
      device->total_parts = data[4] & 0xF;
      if (device->total_parts > 1) {
        device->multipart = true;
      } else {
        device->multipart = false;
      }
    }

    memcpy(device->input_buffer + device->rx_msg_header.length, data + 6, data[5]);
    device->rx_msg_header.length += data[5] - 2;

    // LOG_INF("LoRa RX RSSI: %d dBm, SNR: %d dB", rssi, snr);
    if (current_part == device->total_parts) {
      device->multipart = false;
      device->rx_msg_header.data = device->input_buffer;
      uint16_t checksum = fletcher_checksum_calculation(device->input_buffer, device->rx_msg_header.length);
      if ((device->input_buffer[device->rx_msg_header.length] | device->input_buffer[device->rx_msg_header.length + 1]
                                                                    << 8) == checksum) {
        device->rx_msg_header.checksum_passed = true;
      } else {
        device->rx_msg_header.checksum_passed = false;
      }

      device->handle_message();
      if (device->registered_ && device->rx_msg_header.msg_id) {
        device->msg_cb_(device->rx_net_header, device->rx_msg_header);
      }
    }
  }

  void enable_tx() {
    lora_recv_async(lora_dev_, NULL, NULL);
    config.tx = true;
    int ret = lora_config(lora_dev_, &config);
    if (ret < 0) {
      LOG_ERR("LoRa TX config failed %d", ret);
      return;
    }
  }

  void enable_rx() {
    lora_recv_async(lora_dev_, NULL, NULL);
    config.tx = false;
    int ret = lora_config(lora_dev_, &config);
    if (ret < 0) {
      LOG_ERR("LoRa RX config failed %d", ret);
      return;
    }

    lora_recv_async(lora_dev_, lora_receive_cb, this);
  }

  static uint16_t fletcher_checksum_calculation(uint8_t *buffer, uint8_t data_length) {
    uint8_t byte1{};
    uint8_t byte2{};

    for (int i = 0; i < data_length; i++) {
      byte1 += buffer[i];
      byte2 += byte1;
    }
    return byte1 << 8 | byte2;
  }
};
