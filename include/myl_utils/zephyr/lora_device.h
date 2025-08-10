#pragma once

#include <errno.h>
#include <myl_utils/zephyr/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "stdint.h"

#ifndef CONFIG_LORA
#error Enable CONFIG_MYL_UTILS_LORA in kconfig (prj.conf)
#endif

#define MAX_DATA_LENGTH 200

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

struct lora_id {
  uint32_t mac_id;
  uint8_t node_id;
};

#define MESSAGE_TIME_MS 100

typedef void (*lora_msg_cb)(const struct network_header, const struct message_header);

class LoraDevice {
 public:
  enum class NETWORK_MSG_IDS : uint8_t { BROADCAST, REGISTER, REGISTER_REPLY, NODE_REQUEST, NODE_REPLY };
  LoraDevice(const struct device *const lora_dev, uint8_t net_id, lora_msg_cb msg_cb, bool isServer)
      : lora_dev_{lora_dev}, net_id_{net_id}, msg_cb_{msg_cb}, isServer_{isServer} {
    DECLARE_MYL_UTILS_LOG();
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

    if (!isServer_) {
      config.tx = false;
      rx_message_time = 2000;
    }

    int ret = lora_config(lora_dev_, &config);
    if (ret < 0) {
      LOG_ERR("LoRa config failed %d", ret);
    }
  }

  lora_id GetLoraId() { return {.mac_id = mac_id_, .node_id = node_id_}; }

 protected:
  size_t rx_message_time{MESSAGE_TIME_MS};
  network_header rx_net_header;
  message_header rx_msg_header;
  uint8_t node_id_{};
  uint32_t mac_id_{};
  bool registered_{};

  bool Transmit(uint8_t dest_id, uint8_t msg_id, uint8_t *msg_data, uint8_t msg_length, bool forced = false) {
    DECLARE_MYL_UTILS_LOG();
    if (msg_id && !registered_ && !forced) {
      LOG_INF("Can't transmit non-network messages before registration");
      return false;
    }

    output_buffer[0] = net_id_;
    output_buffer[1] = node_id_;
    output_buffer[2] = dest_id;
    output_buffer[3] = msg_id;

    uint16_t checksum = FletcherChecksumCalculation(msg_data, msg_length);

    uint8_t total_parts = (msg_length / MAX_DATA_LENGTH) + 1;
    if (!isServer_) {
      EnableTx();
    }
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
        if (!isServer_) {
          EnableRx();
        }
        return false;
      }
    }
    if (!isServer_) {
      EnableRx();
    }
    return true;
  };

  void Receive() {
    int ret;
    if (isServer_) {
      if (!EnableRx()) {
        return;
      }
    }

    int16_t rssi;
    int8_t snr;
    uint8_t *data = raw_rx_buffer;
    DECLARE_MYL_UTILS_LOG();
    LOG_DBG("Starting Receive");
    ret = lora_recv(lora_dev_, data, ARRAY_SIZE(raw_rx_buffer), K_MSEC(rx_message_time), &rssi, &snr);
    if (ret < 0) {
      if (ret == -11) {
        return;
      }
      LOG_INF("RX Error %d", ret);
      for (int i = 0; i < 100; i++) {
        raw_rx_buffer[i] = 0;
      }
      return;
    }

    if (isServer_) {
      if (!EnableTx()) {
        return;
      }
    }

    uint8_t current_part = data[4] >> 4;
    if (multipart) {
      if (rx_net_header.network == data[0] && rx_net_header.dest == data[2] && rx_net_header.src == data[1]) {
        if (rx_msg_header.msg_id != data[3]) {
          multipart = false;
          return;
        }
      }
    } else {
      rx_msg_header.length = 0;
      rx_net_header.network = data[0];
      rx_net_header.src = data[1];
      rx_net_header.dest = data[2];

      rx_msg_header.msg_id = data[3];
      total_parts = data[4] & 0xF;
      if (total_parts > 1) {
        multipart = true;
      } else {
        multipart = false;
      }
    }

    memcpy(input_buffer + rx_msg_header.length, data + 6, data[5]);
    rx_msg_header.length += data[5] - 2;

    // LOG_INF("LoRa RX RSSI: %d dBm, SNR: %d dB", rssi, snr);
    if (current_part == total_parts) {
      multipart = false;
      rx_msg_header.data = input_buffer;
      uint16_t checksum = FletcherChecksumCalculation(input_buffer, rx_msg_header.length);
      if ((input_buffer[rx_msg_header.length] | input_buffer[rx_msg_header.length + 1] << 8) == checksum) {
        rx_msg_header.checksum_passed = true;
      } else {
        rx_msg_header.checksum_passed = false;
      }

      HandleMessage();
      if (registered_ && rx_msg_header.msg_id) {
        msg_cb_(rx_net_header, rx_msg_header);
      }
    }
  }

 private:
  const struct device *lora_dev_;
  uint8_t net_id_;
  lora_msg_cb msg_cb_;
  bool isServer_{};

  bool reply = false;

  uint8_t output_buffer[256];
  uint8_t raw_rx_buffer[255];
  uint8_t input_buffer[2048];
  bool multipart = false;
  uint8_t total_parts;
  bool transmitting_;

  struct lora_modem_config config;

  virtual void HandleMessage() = 0;

  bool EnableTx() {
    DECLARE_MYL_UTILS_LOG();
    config.tx = true;
    int ret = lora_config(lora_dev_, &config);
    if (ret < 0) {
      LOG_ERR("LoRa TX config failed %d", ret);
      return false;
    }
    return true;
  }

  bool EnableRx() {
    DECLARE_MYL_UTILS_LOG();
    config.tx = false;
    int ret = lora_config(lora_dev_, &config);
    if (ret < 0) {
      LOG_ERR("LoRa RX config failed %d", ret);
      return false;
    }
    return true;
  }

  static uint16_t FletcherChecksumCalculation(uint8_t *buffer, uint8_t data_length) {
    uint8_t byte1{};
    uint8_t byte2{};

    for (int i = 0; i < data_length; i++) {
      byte1 += buffer[i];
      byte2 += byte1;
    }
    return byte1 << 8 | byte2;
  }
};
