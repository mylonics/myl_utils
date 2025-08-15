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

#ifndef CONFIG_LORA_MODULE_BACKEND_LORA_BASICS_MODEM
#error Enable CONFIG_LORA_MODULE_BACKEND_LORA_BASICS_MODEM in kconfig (prj.conf)
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

    uint8_t total_parts = (msg_length + MAX_DATA_LENGTH - 1) / MAX_DATA_LENGTH;  // ceil div, handles exact multiples

    EnableTx();
    for (int i = 0; i < total_parts; i++) {
      output_buffer[4] = (uint8_t)(((i + 1) << 4) | (total_parts & 0x0F));

      size_t offset = (size_t)i * MAX_DATA_LENGTH;
      uint8_t data_size = 0;
      if (offset < msg_length) {
        size_t remaining = msg_length - offset;
        data_size = (uint8_t)(remaining > MAX_DATA_LENGTH ? MAX_DATA_LENGTH : remaining);
      }

      output_buffer[5] = data_size;
      memcpy(output_buffer + 6, msg_data + (i * MAX_DATA_LENGTH), data_size);

      // need to add checksum on last part
      if ((i + 1) == total_parts) {
        // append 2 checksum bytes after payload
        output_buffer[5] = (uint8_t)(output_buffer[5] + 2);
        output_buffer[6 + data_size] = (uint8_t)(checksum & 0xFF);
        output_buffer[6 + data_size + 1] = (uint8_t)((checksum >> 8) & 0xFF);
      }

      // bounds check to avoid sending more than buffer
      size_t to_send = 6 + output_buffer[5];
      if (to_send > sizeof(output_buffer)) {
        LOG_ERR("LoRa Send: packet too large to send %d", (int)to_send);
        if (!isServer_) {
          EnableRx();
        }
        return false;
      }

      int ret = lora_send(lora_dev_, output_buffer, (size_t)to_send);
      if (ret < 0) {
        LOG_ERR("LoRa Send failed %d msg id %d first byte %d", ret, msg_id, msg_data[0]);
        if (!isServer_) {
          EnableRx();
        }
        return false;
      } else {
        LOG_DBG("LoRa Send succeeded msg id %d first byte %d", msg_id, msg_data[0]);
      }
    }
    if (!isServer_) {
      EnableRx();
    }
    return true;
  };

  void Receive() {
    if (!EnableRx()) {
      return;
    }

    DECLARE_MYL_UTILS_LOG();
    int16_t rssi;
    int8_t snr;
    bool assembling = true;
    bool got_any = false;
    bool final_part = false;
    uint8_t expected_parts = 0;
    uint8_t next_part_index = 1;
    rx_msg_header.length = 0;  // reset for new message

    while (assembling) {
      uint8_t *data = raw_rx_buffer;
      int ret = lora_recv(lora_dev_, data, ARRAY_SIZE(raw_rx_buffer), K_MSEC(rx_message_time), &rssi, &snr);
      if (ret < 0) {
        if (ret == -11) {  // timeout
          // If we already started assembling and timed out mid-message, abort
          if (got_any) {
            LOG_WRN("RX timeout mid-multipart (got %d bytes)", rx_msg_header.length);
          }
          break;
        }
        LOG_INF("RX Error %d", ret);
        break;
      }
      if (ret < 6) {
        LOG_DBG("Short RX %d", ret);
        continue;  // keep listening within loop
      }

      uint8_t part_index = data[4] >> 4;  // 1-based
      uint8_t total_parts = data[4] & 0x0F;
      uint8_t part_total_len = data[5];

      // Validate frame length vs declared payload
      if ((size_t)(6 + part_total_len) > (size_t)ret) {
        LOG_ERR("LoRa RX: declared part length %d exceeds received bytes %d", part_total_len, ret);
        break;
      }

      if (!got_any) {
        // First fragment: capture headers
        rx_net_header.network = data[0];
        rx_net_header.src = data[1];
        rx_net_header.dest = data[2];
        rx_msg_header.msg_id = data[3];
        expected_parts = total_parts ? total_parts : 1;  // guard zero
        next_part_index = 1;
        got_any = true;
      } else {
        // Consistency check for subsequent fragments
        if (rx_net_header.network != data[0] || rx_net_header.src != data[1] || rx_net_header.dest != data[2] ||
            rx_msg_header.msg_id != data[3] || expected_parts != total_parts) {
          LOG_ERR("LoRa RX: fragment header mismatch, aborting multipart");
          break;
        }
      }

      if (part_index != next_part_index) {
        LOG_ERR("LoRa RX: unexpected part index %d expected %d", part_index, next_part_index);
        break;
      }

      bool is_final = (part_index == expected_parts);
      if (!is_final) {
        if (part_total_len == 0) {
          LOG_ERR("LoRa RX: zero-length non-final part");
          break;
        }
        if ((size_t)rx_msg_header.length + part_total_len > sizeof(input_buffer)) {
          LOG_ERR("LoRa RX: overflow assembling non-final part");
          break;
        }
        memcpy(input_buffer + rx_msg_header.length, data + 6, part_total_len);
        rx_msg_header.length += part_total_len;
      } else {
        // final part includes checksum at tail (2 bytes)
        if (part_total_len < 2) {
          LOG_ERR("LoRa RX: final part too short for checksum");
          break;
        }
        uint8_t payload_bytes = (uint8_t)(part_total_len - 2);
        if ((size_t)rx_msg_header.length + payload_bytes > sizeof(input_buffer)) {
          LOG_ERR("LoRa RX: overflow assembling final part");
          break;
        }
        memcpy(input_buffer + rx_msg_header.length, data + 6, payload_bytes);
        rx_msg_header.length += payload_bytes;
        if ((size_t)rx_msg_header.length + 2 <= sizeof(input_buffer)) {
          input_buffer[rx_msg_header.length] = data[6 + payload_bytes];
          input_buffer[rx_msg_header.length + 1] = data[6 + payload_bytes + 1];
        } else {
          LOG_ERR("LoRa RX: no space to store checksum");
          break;
        }
        final_part = true;
      }

      if (is_final) {
        assembling = false;  // done
      } else {
        next_part_index++;
        // continue loop to get next fragment quickly (no mode switch yet)
      }
    }

    if (isServer_) {
      EnableTx();
    }

    if (!got_any || !final_part) {
      // Nothing complete assembled; if server we still need to restore TX
      return;
    }

    // Validate checksum for completed packet
    rx_msg_header.data = input_buffer;
    if ((size_t)rx_msg_header.length + 1 < sizeof(input_buffer)) {
      uint16_t checksum = FletcherChecksumCalculation(input_buffer, (uint8_t)rx_msg_header.length);
      uint16_t stored =
          (uint16_t)input_buffer[rx_msg_header.length] | ((uint16_t)input_buffer[rx_msg_header.length + 1] << 8);
      rx_msg_header.checksum_passed = (stored == checksum);
    } else {
      rx_msg_header.checksum_passed = false;
    }

    HandleMessage();
    if (registered_ && rx_msg_header.msg_id) {
      msg_cb_(rx_net_header, rx_msg_header);
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

  bool SetTxRx(bool tx) {
    if (config.tx != tx) {
      config.tx = tx;

      int ret = lora_config(lora_dev_, &config);
      if (ret < 0) {
        DECLARE_MYL_UTILS_LOG();
        LOG_ERR("LoRa TX:%d config failed %d", config.tx, ret);
        config.tx = !config.tx;
        return false;
      }
    }
    return true;
  }

  bool EnableTx() { return SetTxRx(true); }

  bool EnableRx() { return SetTxRx(false); }

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
