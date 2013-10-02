// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
//  Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

// the log number of the last log we started
static uint16_t log_number;

struct PACKED data_packet {
    LOG_PACKET_HEADER;
    float     time_since_birth;
    int16_t   packetNumber;
    float     target_ctt;
    float     cc;
    float     roll;
    float     pitch;
    float     lat;
    float     lon;
    float     sog;
    float     cog;
    float     depth;
    float     adc0;
    float     adc1;
    float     adc2;
    float     adc4;
};

#define LOG_DATA_MSG 1

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_DATA_MSG, sizeof(data_packet),       
      "KTH", "fhfffffffffffff",        "T,PN,tCTT,CC,Rll,Ptch,Lat,Lon,SOG,COG,Dpth,A0,A1,A2,A4" }
};


static void start_new_log(void)
{
    DataFlash.StartNewLog(sizeof(log_structure)/sizeof(log_structure[0]), log_structure);
    log_number = DataFlash.find_last_log();
    hal.console->printf_P(PSTR("Started log number %u\n"), 
                          (unsigned)log_number);
}

static void erase_logs(void)
{
    hal.console->printf_P(PSTR("    Erasing... Data flash card (be patient!)  "));
    DataFlash.EraseAll();
    hal.console->printf_P(PSTR("done!\n"));
    start_new_log();
}


//-------------------------------------------------------------------------------
static void setup_Flash()
{
    DataFlash.Init();
    // DataFlash initialization    
    DataFlash.ShowDeviceInfo(hal.console);
    if (DataFlash.NeedErase()) {
        erase_logs();
    }
    start_new_log();
}


//-------------------------------------------------------------------------------
static void write_a_row_to_flash()
{
  if ((time_ms-last_log_ms) <= 1000)  {
      return;
  }
  last_log_ms = time_ms;
  packets_written = packets_written+1;
  struct data_packet pkt = {
      LOG_PACKET_HEADER_INIT(LOG_DATA_MSG),
      time_since_birth  : (float) ((time_ms-birth_ms)/1000.0),
      packetNumber      : (int16_t) packets_written,
      target_ctt        : (float) target_ctt,
      cc                : (float) heading,
      roll              : (float) roll,
      pitch             : (float) pitch,
      lat               : (float) current_pos.lat*180/pi,
      lon               : (float) current_pos.lon*180/pi,
      sog               : (float) gps.sog,
      cog               : (float) gps.cog,
      depth             : (float) depth,
      adc0              : (float) adc0,
      adc1              : (float) adc1,
      adc2              : (float) adc2,
      adc4              : (float) adc4
  };
  
  DataFlash.WriteBlock(&pkt, sizeof(pkt));
  hal.console->printf_P(PSTR("adc0=%f , adc1=%f , adc2=%f , adc4=%f,\n"),pkt.adc0,pkt.adc1,pkt.adc2,pkt.adc4);
}

//-------------------------------------------------------------------------------
void flash_read_all_packets()
{
  hal.console->printf_P(PSTR("\nWill now read Flash card \n"));
  DataFlash.ListAvailableLogs(hal.console);

  uint16_t num_logs = DataFlash.get_num_logs();
  uint16_t start_page, end_page;

  if (num_logs == 0) {
      hal.console->printf_P(PSTR("No logs available - starting a new log\n"));
      DataFlash.StartNewLog(sizeof(log_structure)/sizeof(log_structure[0]), log_structure);
      return;
  }

  DataFlash.get_log_boundaries(log_number, start_page, end_page);

  hal.console->printf_P(PSTR("\nReading Flash card lognum=%u from page %u to %u\n"),
                        (unsigned)log_number, (unsigned)start_page, (unsigned)end_page);

  DataFlash.LogReadProcess(log_number, start_page, end_page, 
                           sizeof(log_structure)/sizeof(log_structure[0]),
                           log_structure, 
                           NULL,
                           hal.console);
}
//-------------------------------------------------------------------------------




