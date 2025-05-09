/*
 * E131Input.cpp - Code to wrap ESPAsyncE131 for input
 *
 * Project: ESPixelStick - An ESP8266 / ESP32 and E1.31 based pixel driver
 * Copyright (c) 2021, 2025 Shelby Merrick
 * http://www.forkineye.com
 *
 *  This program is provided free for you to use in any way that you wish,
 *  subject to the laws and regulations where you are using it.  Due diligence
 *  is strongly suggested before using this code.  Please give credit where due.
 *
 *  The Author makes no warranty of any kind, express or implied, with regard
 *  to this program or the documentation contained in this document.  The
 *  Author shall not be liable in any event for incidental or consequential
 *  damages in connection with, or arising out of, the furnishing, performance
 *  or use of these programs.
 *
 */

#include "input/InputE131.hpp"
#include "network/NetworkMgr.hpp"
#include <Arduino.h>  // für millis(), pinMode(), digitalWrite()

//-----------------------------------------------------------------------------
// Konstruktor / Destruktor
c_InputE131::c_InputE131 (c_InputMgr::e_InputChannelIds NewInputChannelId,
                          c_InputMgr::e_InputType       NewChannelType,
                          uint32_t                        BufferSize) :
    c_InputCommon(NewInputChannelId, NewChannelType, BufferSize)
{
    e131 = new ESPAsyncE131 (0);
    memset((void*)UniverseArray, 0x00, sizeof(UniverseArray));
}

c_InputE131::~c_InputE131()
{
}

//-----------------------------------------------------------------------------
void c_InputE131::Begin ()
{
    do // once
    {
        validateConfiguration();

        e131->registerCallback((void*)this, [](e131_packet_t* Packet, void* pThis) {
            ((c_InputE131*)pThis)->ProcessIncomingE131Data(Packet);
        });

        NetworkStateChanged(NetworkMgr.IsConnected(), false);

        // GPIO für DE initialisieren
        pinMode(TX_ENABLE_PIN, OUTPUT);
        digitalWrite(TX_ENABLE_PIN, HIGH);  // DMX-Ausgang standardmäßig aktiviert

        HasBeenInitialized = true;
    } while (false);
}

//-----------------------------------------------------------------------------
void c_InputE131::GetConfig (JsonObject & jsonConfig)
{
    JsonWrite(jsonConfig, CN_universe,       startUniverse);
    JsonWrite(jsonConfig, CN_universe_limit, ChannelsPerUniverse);
    JsonWrite(jsonConfig, CN_universe_start, FirstUniverseChannelOffset);
    JsonWrite(jsonConfig, CN_port,           PortId);
}

//-----------------------------------------------------------------------------
void c_InputE131::GetStatus (JsonObject & jsonStatus)
{
    JsonObject e131Status = jsonStatus[(char*)F("e131")].to<JsonObject>();
    JsonWrite(e131Status, CN_id,         InputChannelId);
    JsonWrite(e131Status, CN_unifirst,   startUniverse);
    JsonWrite(e131Status, CN_unilast,    LastUniverse);
    JsonWrite(e131Status, CN_unichanlim, ChannelsPerUniverse);

    JsonWrite(e131Status, CN_num_packets,   e131->stats.num_packets);
    JsonWrite(e131Status, CN_last_clientIP, uint32_t(e131->stats.last_clientIP));

    JsonArray e131UniverseStatus = e131Status[(char*)CN_channels].to<JsonArray>();
    uint32_t TotalErrors = 0;
    for (auto & CurrentUniverse : UniverseArray)
    {
        JsonObject e131CurrentUniverseStatus = e131UniverseStatus.add<JsonObject>();
        JsonWrite(e131CurrentUniverseStatus, CN_errors, CurrentUniverse.SequenceErrorCounter);
        TotalErrors += CurrentUniverse.SequenceErrorCounter;
    }
    JsonWrite(e131Status, CN_packet_errors, TotalErrors);
}

//-----------------------------------------------------------------------------
void c_InputE131::Process ()
{
    // Watchdog prüfen und GPIO entsprechend setzen
    uint32_t now = millis();
    if (now - lastPacketTime > PACKET_TIMEOUT_MS)
    {
        digitalWrite(TX_ENABLE_PIN, LOW);   // kein sACN → DMX deaktivieren
    }
    else
    {
        digitalWrite(TX_ENABLE_PIN, HIGH);  // sACN aktiv → DMX aktivieren
    }
}

//-----------------------------------------------------------------------------
void c_InputE131::ProcessIncomingE131Data (e131_packet_t * packet)
{
    // Watchdog zurücksetzen
    lastPacketTime = millis();

    if ((0 == InputDataBufferSize) || !IsInputChannelActive)
    {
        return;
    }

    uint16_t CurrentUniverseId = ntohs(packet->universe);
    uint8_t *E131Data = packet->property_values + 1;

    if ((startUniverse <= CurrentUniverseId) && (LastUniverse >= CurrentUniverseId))
    {
        Universe_t& CurrentUniverse = UniverseArray[CurrentUniverseId - startUniverse];

        // Sequenz-Fehler-Handling
        if (packet->sequence_number != CurrentUniverse.SequenceNumber)
        {
            if (0 != packet->sequence_number)
            {
                CurrentUniverse.SequenceErrorCounter++;
            }
            CurrentUniverse.SequenceNumber = packet->sequence_number;
        }
        ++CurrentUniverse.SequenceNumber;

        uint32_t NumBytesOfE131Data = uint32_t(ntohs(packet->property_value_count) - 1);
        OutputMgr.WriteChannelData(
            CurrentUniverse.DestinationOffset,
            min(CurrentUniverse.BytesToCopy, NumBytesOfE131Data),
            &E131Data[CurrentUniverse.SourceDataOffset]
        );

        InputMgr.RestartBlankTimer(GetInputChannelId());
    }
}

//-----------------------------------------------------------------------------
void c_InputE131::SetBufferInfo (uint32_t BufferSize)
{
    InputDataBufferSize = BufferSize;

    if (HasBeenInitialized)
    {
        HasBeenInitialized = false;
        Begin();
    }

    SetBufferTranslation();
}

//-----------------------------------------------------------------------------
void c_InputE131::SetBufferTranslation ()
{
    uint32_t InputOffset = FirstUniverseChannelOffset - 1;
    uint32_t DestinationOffset = 0;
    uint32_t BytesLeftToMap = InputDataBufferSize;
    uint32_t BytesInUniverse = ChannelsPerUniverse - InputOffset;

    for (auto& CurrentUniverse : UniverseArray)
    {
        uint16_t BytesInThisUniverse = min(BytesInUniverse, BytesLeftToMap);
        CurrentUniverse.DestinationOffset = DestinationOffset;
        CurrentUniverse.BytesToCopy     = BytesInThisUniverse;
        CurrentUniverse.SourceDataOffset = InputOffset;
        CurrentUniverse.SequenceErrorCounter = 0;
        CurrentUniverse.SequenceNumber = 0;

        DestinationOffset += BytesInThisUniverse;
        BytesLeftToMap    -= BytesInThisUniverse;
        BytesInUniverse    = ChannelsPerUniverse;
        InputOffset        = 0;
    }

    if (BytesLeftToMap != 0)
    {
        logcon(String(F("ERROR: Universe configuration is too small to fill output buffer. Outputs have been truncated.")));
    }
}

//-----------------------------------------------------------------------------
bool c_InputE131::SetConfig (ArduinoJson::JsonObject& jsonConfig)
{
    ESPAsyncE131PortId OldPortId = PortId;

    setFromJSON(startUniverse,               jsonConfig, CN_universe);
    setFromJSON(ChannelsPerUniverse,         jsonConfig, CN_universe_limit);
    setFromJSON(FirstUniverseChannelOffset,  jsonConfig, CN_universe_start);
    setFromJSON(PortId,                      jsonConfig, CN_port);

    if ((OldPortId != PortId) && (ESPAsyncE131Initialized))
    {
        RequestReboot(100000);
        logcon(String(F("Requesting reboot on change of UDP port.")));
    }

    validateConfiguration();
    GetConfig(jsonConfig);
    return true;
}

//-----------------------------------------------------------------------------
void c_InputE131::validateConfiguration ()
{
    if (startUniverse < 1) startUniverse = 1;
    if (ChannelsPerUniverse > UNIVERSE_MAX || ChannelsPerUniverse < 1)
        ChannelsPerUniverse = UNIVERSE_MAX;
    if (FirstUniverseChannelOffset < 1)
        FirstUniverseChannelOffset = 1;
    else if (FirstUniverseChannelOffset > ChannelsPerUniverse)
        FirstUniverseChannelOffset = ChannelsPerUniverse - 1;

    uint32_t span = FirstUniverseChannelOffset + InputDataBufferSize - 1;
    if (span % ChannelsPerUniverse)
        LastUniverse = startUniverse + span / ChannelsPerUniverse;
    else
        LastUniverse = startUniverse + span / ChannelsPerUniverse - 1;

    SetBufferTranslation();
}

//-----------------------------------------------------------------------------
void c_InputE131::NetworkStateChanged (bool IsConnected)
{
    NetworkStateChanged(IsConnected, false);
}

void c_InputE131::NetworkStateChanged (bool IsConnected, bool ReBootAllowed)
{
    if (IsConnected)
    {
        if (!e131->begin(e131_listen_t::E131_MULTICAST, PortId, startUniverse, LastUniverse - startUniverse + 1))
            logcon(String(CN_stars) + F(" E1.31 MULTICAST INIT FAILED ") + CN_stars);

        if (!e131->begin(e131_listen_t::E131_UNICAST, PortId, startUniverse, LastUniverse - startUniverse + 1))
            logcon(String(CN_stars) + F(" E1.31 UNICAST INIT FAILED ") + CN_stars);

        logcon(String(F("Listening for ")) + InputDataBufferSize +
               F(" channels from Universe ") + startUniverse +
               F(" to ") + LastUniverse +
               F(" on port ") + PortId);

        ESPAsyncE131Initialized = true;
    }
    else if (ReBootAllowed)
    {
        RequestReboot(100000);
        logcon(String(F("Input requesting reboot on loss of WiFi connection.")));
    }
}
