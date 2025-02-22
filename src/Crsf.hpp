#include <Arduino.h>

#include "Crc8.h"

namespace crsf_ns {

  struct RcData {
  bool failsafe = true ;
  uint16_t channels[16] = {0};
};  

typedef void (*RcCallback)(RcData &);

const uint16_t crsf_rc_channel_min = 172;
const uint16_t crsf_rc_channel_max = 1811;
const uint16_t crsf_rc_channel_center = 992;
const uint16_t crsf_rc_channel_range = crsf_rc_channel_max - crsf_rc_channel_min;

// returns float [0, 1] from crsf rc channel value
float crsf_rc_channel_to_float(uint16_t value) {
  using namespace crsf_ns;

  return (value -  crsf_rc_channel_center) * 2.0 / crsf_rc_channel_range;
}

}


namespace internal_to_crsf {

void print_bytes(uint8_t *bytes, uint8_t size) {
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%02X ", bytes[i]);
  }
}

const int crsf_sync_byte = 0xC8;

enum CrsfFrameType {
  CRSF_FRAMETYPE_GPS = 0x02,
  CRSF_FRAMETYPE_VARIO_SENSOR = 0x07,
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
  CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
  CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
  CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  // Extended Header Frames, range: 0x28 to 0x96
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
  CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
  CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
  CRSF_FRAMETYPE_COMMAND = 0x32,
  // MSP commands
  CRSF_FRAMETYPE_MSP_REQ =
      0x7A,  // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B,   // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX
                                    // outbound telemetry buffer limit)
  CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D,  // displayport control command
};



struct CrsfRcChannelsPacked {
  uint16_t channel_01 : 11;
  uint16_t channel_02 : 11;
  uint16_t channel_03 : 11;
  uint16_t channel_04 : 11;
  uint16_t channel_05 : 11;
  uint16_t channel_06 : 11;
  uint16_t channel_07 : 11;
  uint16_t channel_08 : 11;
  uint16_t channel_09 : 11;
  uint16_t channel_10 : 11;
  uint16_t channel_11 : 11;
  uint16_t channel_12 : 11;
  uint16_t channel_13 : 11;
  uint16_t channel_14 : 11;
  uint16_t channel_15 : 11;
  uint16_t channel_16 : 11;
} __attribute__((packed));

struct CrsfLinkStatistics {
  uint8_t up_rssi_ant1;       // Uplink RSSI Antenna 1 (dBm * -1)
  uint8_t up_rssi_ant2;       // Uplink RSSI Antenna 2 (dBm * -1)
  uint8_t up_link_quality;    // Uplink Package success rate / Link quality (%)
  int8_t up_snr;              // Uplink SNR (dB)
  uint8_t active_antenna;     // number of currently best antenna
  uint8_t rf_profile;         // enum {4fps = 0 , 50fps, 150fps}
  uint8_t up_rf_power;        // enum {0mW = 0, 10mW, 25mW, 100mW,
                              // 500mW, 1000mW, 2000mW, 250mW, 50mW}
  uint8_t down_rssi;          // Downlink RSSI (dBm * -1)
  uint8_t down_link_quality;  // Downlink Package success rate / Link
                              // quality (%)
  int8_t down_snr;            // Downlink SNR (dB)
};

struct CrsfBatterySensor {
  uint16_t voltage;        // Voltage (LSB = 10 µV)
  uint16_t current;        // Current (LSB = 10 µA)
  uint32_t capacity_used;  // Capacity used (mAh)
  uint8_t remaining;       // Battery remaining (percent)
} __attribute__((packed));

struct CrsfAttitude {
  int16_t pitch;  // Pitch angle (LSB = 100 µrad)
  int16_t roll;   // Roll angle  (LSB = 100 µrad)
  int16_t yaw;    // Yaw angle   (LSB = 100 µrad)
} __attribute__((packed));

struct CrsfGPS {
  int32_t latitude;      // degree / 10`000`000
  int32_t longitude;     // degree / 10`000`000
  uint16_t groundspeed;  // km/h / 100
  uint16_t heading;      // degree / 100
  uint16_t altitude;     // meter - 1000m offset
  uint8_t satellites;    // # of sats in view
};

}  // namespace internal_to_crsf


class Crsf {

 private:
  crsf_ns::RcCallback rc_callback;


  // for input state machine
  uint8_t length_byte = 0;
  uint8_t payload[64];
  uint8_t payload_length = 0;
  uint8_t frame_type = 0;

  crsf_ns::RcData rc_data;
  unsigned long last_rc_packet_time = 0;

 public:
   void set_rc_callback(crsf_ns::RcCallback callback) {
     rc_callback = callback;
   }
 
   enum PackateState {
     awaiting_start,
     awaiting_device_address,
     awaiting_length,
     awaiting_frame_type,
     awaiting_payload,
     awaiting_crc
   } packet_state;
 
 
   void process_payload() {
    using namespace internal_to_crsf;
     CrsfFrameType frame_type = static_cast<CrsfFrameType>(payload[0]);
     void *data = payload + 1;
     if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
       CrsfRcChannelsPacked *rc_channels = (CrsfRcChannelsPacked *)data;
       rc_data.failsafe = false;
       rc_data.channels[0] = rc_channels->channel_01;
       rc_data.channels[1] = rc_channels->channel_02;
       rc_data.channels[2] = rc_channels->channel_03;
       rc_data.channels[3] = rc_channels->channel_04;
       rc_data.channels[4] = rc_channels->channel_05;
       rc_data.channels[5] = rc_channels->channel_06;
       rc_data.channels[6] = rc_channels->channel_07;
       rc_data.channels[7] = rc_channels->channel_08;
       rc_data.channels[8] = rc_channels->channel_09;
       rc_data.channels[9] = rc_channels->channel_10;
       rc_data.channels[10] = rc_channels->channel_11;
       rc_data.channels[11] = rc_channels->channel_12;
       rc_data.channels[12] = rc_channels->channel_13;
       rc_data.channels[13] = rc_channels->channel_14;
       rc_data.channels[14] = rc_channels->channel_15;
       rc_data.channels[15] = rc_channels->channel_16;
 
        if(rc_callback) {
          rc_callback(rc_data);
        }
        last_rc_packet_time = millis();
     } else if (frame_type == CRSF_FRAMETYPE_LINK_STATISTICS) {
       CrsfLinkStatistics *link_stats = (CrsfLinkStatistics *)data;
       // Serial.printf("Link Stats: up_rssi_ant1: %d up_rssi_ant2: %d
       // up_link_quality: %d up_snr: %d active_antenna: %d rf_profile: %d
       // up_rf_power: %d down_rssi: %d down_link_quality: %d down_snr: %d\n",
       //   link_stats->up_rssi_ant1, link_stats->up_rssi_ant2,
       //   link_stats->up_link_quality, link_stats->up_snr,
       //   link_stats->active_antenna, link_stats->rf_profile,
       //   link_stats->up_rf_power, link_stats->down_rssi,
       //   link_stats->down_link_quality, link_stats->down_snr);
     } else {
       Serial.printf("Frame type: 0x%02X\n", frame_type);
       Serial.printf("Payload: ");
       internal_to_crsf::print_bytes(payload, payload_length);
       Serial.println();
     }
   }
   void process_crsf_byte(uint8_t c) {
    using namespace internal_to_crsf;
 
     if (packet_state == awaiting_start) {
       if (c == crsf_sync_byte) {
         packet_state = awaiting_length;
       } else {
        // todo: this is an protocol error, track them
       }
     }
     else if (packet_state == awaiting_length) {
       length_byte = c;
       if (length_byte > 62) {
         packet_state = awaiting_start;
         return;
       }
       packet_state = awaiting_payload;
       payload_length = 0;
     }
     else if (packet_state == awaiting_payload) {
       payload[payload_length] = c;
       payload_length++;
       const int max_payload_length = sizeof(payload);

       if (payload_length >= length_byte - 1) {
         packet_state = awaiting_crc;
       }
     } else if (packet_state == awaiting_crc) {
       uint8_t crc = c;
       uint8_t offset = 2;
       uint8_t calculated_crc = crc8(payload, payload_length);
       if (crc == calculated_crc) {
         process_payload();
       } else {
         Serial.printf(
             "CRC FAIL, was 0x%02X expected 0x%02X length 0x%02X "
             "payload_length 0x%02X\n",
             calculated_crc, crc, length_byte, payload_length);
         Serial.printf("Payload: ");
         print_bytes(payload, payload_length);
         Serial.println();
       }
       Serial.flush();
       //  while(1) delay(1000);
       packet_state = awaiting_start;
     }
   }


 public:
  HardwareSerial &crsf_serial;
  Crsf(HardwareSerial &serial) : crsf_serial(serial) {}

  void begin() { crsf_serial.begin(420000); }


  void update() {
    serial_read_stats.start();
    bool available = crsf_serial.available();
    serial_read_stats.stop();
    if (available) {
      while (crsf_serial.available()) {
        uint8_t c = crsf_serial.read();
        // Serial.printf("0x%02X ", c);
        process_crsf_byte(c);
        serial_read_stats.start();
        available = crsf_serial.available();
        serial_read_stats.stop();
      }
      // Serial.println();
    }
    if (rc_data.failsafe == false && millis() - last_rc_packet_time > 1000) {
      rc_data.failsafe = true;
      memset(rc_data.channels, 0, sizeof(rc_data.channels));
      if(rc_callback) {
        rc_callback(rc_data);
      }
    }
  }

  void write_crsf_frame(internal_to_crsf::CrsfFrameType frame_type, uint8_t *bytes,
                        uint8_t size) {
    uint8_t payload[64];
    payload[0] = internal_to_crsf::crsf_sync_byte;  // sync byte
    payload[1] =
        size + 2;  // length byte (size of payload + size of crc + frame type)
    payload[2] = frame_type;
    for (int i = 0; i < size; i++) {
      payload[i + 3] = bytes[i];
    }
    payload[payload[1] + 1] = crc8(payload + 2, payload[1] - 1);
    if (crsf_serial.availableForWrite() < payload[1] + 2) {
      Serial.println("crsf serial write buffer full, skipped write_crsf_frame");
      return;
    }
    crsf_serial.write(payload, payload[1] + 2);
    //  Serial.printf("Sent frame ");
    //   print_bytes(payload, payload[1]+2);
    //   Serial.println();
  }

  void send_flight_mode(const char *mode) {
    uint8_t length = strlen(mode) + 1;  // include null terminator

    write_crsf_frame(internal_to_crsf::CRSF_FRAMETYPE_FLIGHT_MODE, (uint8_t *)mode, length);
  }

  void send_attitude(float pitch_degrees, float roll_degrees, float yaw_degrees) {
    auto degrees_to_decirad = [](float degrees) -> int16_t {
      // degrees must be [-180, 180]
      while (degrees < -180) degrees += 360;
      while (degrees > 180) degrees -= 360;

      float radians = degrees * PI / 180;
      return radians * 10000;
    };

    internal_to_crsf::CrsfAttitude attitude;
    attitude.pitch = degrees_to_decirad(pitch_degrees);
    attitude.roll = degrees_to_decirad(roll_degrees);
    attitude.yaw = degrees_to_decirad(yaw_degrees);

    char buffer[6];
    buffer[0] = attitude.pitch >> 8 & 0xFF;
    buffer[1] = attitude.pitch & 0xFF;
    buffer[2] = attitude.roll >> 8 & 0xFF;
    buffer[3] = attitude.roll & 0xFF;
    buffer[4] = attitude.yaw >> 8 & 0xFF;
    buffer[5] = attitude.yaw & 0xFF;

    write_crsf_frame(internal_to_crsf::CRSF_FRAMETYPE_ATTITUDE, (uint8_t *)&buffer,
                     sizeof(buffer));
  } 

  void send_battery(float voltage, float current, uint32_t mah_used,
                    uint8_t percent_remaining) {
    internal_to_crsf::CrsfBatterySensor battery_sensor;
    battery_sensor.voltage = voltage * 10;
    battery_sensor.current = current * 10;
    battery_sensor.capacity_used = mah_used;
    battery_sensor.remaining = percent_remaining;

    char buffer[8];
    buffer[0] = battery_sensor.voltage & 0xFF00;
    buffer[1] = battery_sensor.voltage & 0xFF;
    buffer[2] = battery_sensor.current & 0xFF00;
    buffer[3] = battery_sensor.current & 0xFF;
    buffer[4] = (battery_sensor.capacity_used >> 16) & 0xFF;
    buffer[5] = (battery_sensor.capacity_used >> 8) & 0xFF;
    buffer[6] = battery_sensor.capacity_used & 0xFF;
    buffer[7] = battery_sensor.remaining;

    write_crsf_frame(internal_to_crsf::CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t *)&buffer,
                     sizeof(buffer));
  }

  void send_gps(float lat, float lon, float groundspeed, float heading,
                float altitude, uint8_t satellites) {
    internal_to_crsf::CrsfGPS gps;
    uint8_t buffer[15];
    gps.latitude = lat * 10000000;
    gps.longitude = lon * 10000000;
    gps.groundspeed = groundspeed * 100;
    gps.heading = heading * 100;
    gps.altitude = altitude + 1000;
    gps.satellites = satellites;

    buffer[0] = gps.latitude >> 24 & 0xFF;
    buffer[1] = gps.latitude >> 16 & 0xFF;
    buffer[2] = gps.latitude >> 8 & 0xFF;
    buffer[3] = gps.latitude & 0xFF;
    buffer[4] = gps.longitude >> 24 & 0xFF;
    buffer[5] = gps.longitude >> 16 & 0xFF;
    buffer[6] = gps.longitude >> 8 & 0xFF;
    buffer[7] = gps.longitude & 0xFF;
    buffer[8] = gps.groundspeed >> 8 & 0xFF;
    buffer[9] = gps.groundspeed & 0xFF;
    buffer[10] = gps.heading >> 8 & 0xFF;
    buffer[11] = gps.heading & 0xFF;
    buffer[12] = gps.altitude >> 8 & 0xFF;
    buffer[13] = gps.altitude & 0xFF;
    buffer[14] = gps.satellites;

    write_crsf_frame(internal_to_crsf::CRSF_FRAMETYPE_GPS, buffer, sizeof(buffer));
  }
};
