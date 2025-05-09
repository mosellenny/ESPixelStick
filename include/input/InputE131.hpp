// input/InputE131.hpp
#pragma once
/*
 * E131Input.h - Code to wrap ESPAsyncE131 for input
 *
 * Project: ESPixelStick - An ESP8266 / ESP32 and E1.31 based pixel driver
 * Copyright (c) 2021, 2025 Shelby Merrick
 * http://www.forkineye.com
 */

#include "InputCommon.hpp"
#include <ESPAsyncE131.h>
#include <Arduino.h>  // für pinMode(), digitalWrite(), millis()

// Pin für DE am MAX485 (z.B. D2 am Wemos D1 mini)
#ifndef TX_ENABLE_PIN
#  define TX_ENABLE_PIN D2
#endif

class c_InputE131 : public c_InputCommon
{
  private:
    static const uint16_t   UNIVERSE_MAX = 512;
    static const char       ConfigFileName[];
    static const uint8_t    MAX_NUM_UNIVERSES = (OM_MAX_NUM_CHANNELS / UNIVERSE_MAX) + 1;

    ESPAsyncE131  * e131 = nullptr;        ///< ESPAsyncE131

    // Flag, ob E1.31 wirklich initialisiert wurde
    bool            e131Enabled = false;
    // Letzter Pin-Zustand, um nur bei Wechsel digitalWrite aufzurufen
    bool            lastDeState = true;

    /// JSON configuration parameters
    uint16_t    startUniverse              = 1;
    uint16_t    LastUniverse               = 1;
    uint16_t    ChannelsPerUniverse        = 512;
    uint16_t    FirstUniverseChannelOffset = 1;
    ESPAsyncE131PortId PortId              = E131_DEFAULT_PORT;
    bool        ESPAsyncE131Initialized    = false;

    /// from sketch globals
    uint16_t    channel_count = 0;

    struct Universe_t
    {
      uint32_t   DestinationOffset;
      uint32_t   BytesToCopy;
      uint32_t   SourceDataOffset;
      uint8_t    SequenceNumber;
      uint32_t   SequenceErrorCounter;
    };
    Universe_t UniverseArray[MAX_NUM_UNIVERSES];

    void validateConfiguration ();
    void NetworkStateChanged (bool IsConnected, bool RebootAllowed);
    void SetBufferTranslation ();

    // Watchdog für sACN-Pakete
    uint32_t lastPacketTime = 0;
    static const uint32_t PACKET_TIMEOUT_MS = 2000;

  public:
    c_InputE131 (c_InputMgr::e_InputChannelIds NewInputChannelId,
                 c_InputMgr::e_InputType       NewChannelType,
                 uint32_t                        BufferSize);
    virtual ~c_InputE131();

    void Begin ();
    bool SetConfig (JsonObject & jsonConfig);
    void GetConfig (JsonObject & jsonConfig);
    void GetStatus (JsonObject & jsonStatus);
    void Process   ();
    void GetDriverName (String & sDriverName) { sDriverName = "E1.31"; }
    void SetBufferInfo (uint32_t BufferSize);
    void NetworkStateChanged (bool IsConnected);
    bool isShutDownRebootNeeded () { return HasBeenInitialized; }
    void ProcessIncomingE131Data (e131_packet_t *);
};
