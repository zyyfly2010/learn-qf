// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Here is a shange

//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------
//-------------------------------------------------------------------------------
void print_settings(){
   hal.console->printf_P(PSTR("\n-----------------------------------------------------------------------------------------\n"));
   hal.console->printf_P(PSTR("<< Info dump >>\n"));
   hal.console->printf_P(PSTR("Memory free:  %u   (out of 8000 bytes)\n"),(unsigned) memcheck_available_memory());
   
   hal.console->printf_P(PSTR("     Craft type    = '%c'\n"),craft_type);
   hal.console->printf_P(PSTR("     Control mode  = '%c'\n"),ctrl_mode);
   hal.console->printf_P(PSTR("\n"));          
   hal.console->printf_P(PSTR("                   kP          kI         kD      \n"));
   hal.console->printf_P(PSTR("     PID 1  : %10.2f %10.2f %10.2f     \n"),pid_1.kP() ,pid_1.kI() , pid_1.kD() );
   hal.console->printf_P(PSTR("     PID 2  : %10.2f %10.2f %10.2f     \n"),pid_2.kP() ,pid_2.kI() , pid_2.kD() );
   hal.console->printf_P(PSTR("     PID 3  : %10.2f %10.2f %10.2f     \n"),pid_3.kP() ,pid_3.kI() , pid_3.kD() );
   hal.console->printf_P(PSTR("     PID 4  : %10.2f %10.2f %10.2f     \n"),pid_4.kP() ,pid_4.kI() , pid_4.kD() );
   hal.console->printf_P(PSTR("     After mixing pwm_port=%i  pwm_stbd=%i\n\n"), pwm_port , pwm_stbd);
   hal.console->printf_P(PSTR("\n     GPS:   lon=%0.5f, lat=%0.5f, Altitude=%.2fm sog=%.2fm/s cog=%.1f SAT=%d time=%lu status=%i\n"),
                         ToDeg(gps.lon),ToDeg(gps.lat),gps.alt,gps.sog,ToDeg(gps.cog), gps.nsats,  
                         (unsigned long)gps.time,  gps.status);
   hal.console->printf_P(PSTR("     Current position lon=%0.5f, lat=%0.5f  (Could be GPS or dead reconing) \n"),ToDeg(current_pos.lon),ToDeg(current_pos.lat));          
   hal.console->printf_P(PSTR("     IMU: cc =%4.1fdeg    roll=%0.1fdeg   pitch= %0.1fdeg  "),ToDeg(heading),ToDeg(roll),ToDeg(pitch));
   hal.console->printf_P(PSTR("  Acc=%4.2f,%4.2f,%4.2f (norm:%4.2f)  Gyro: %4.3f,%4.3f,%4.3f\n"),accel.x, accel.y, accel.z, accel.length(), gyro.x, gyro.y, gyro.z);
   print_CC_mission();
   print_GPS_mission(); 
   hal.console->printf_P(PSTR("\n     Analogue channels:  adc0: %f, adc1: %f, adc2: %f, adc4: %f, vcc: %f\r\n"),adc0, adc1, adc2, adc4, vcc);  
   hal.console->printf_P(PSTR("       AUV accel_length_crash_threshold = %.2f \n"), accel_length_crash_threshold);
   hal.console->printf_P(PSTR("       AUV gyro_length_crash_threshold  = %.2f \n"), gyro_length_crash_threshold);   
   hal.console->printf_P(PSTR("-----------------------------------------------------------------------------------------\n"));
}


//---------------------------------------------------------------------------
int8_t  menu_print_settings(uint8_t argc, const Menu::arg *argv){
  print_settings();
  return 0;
}

//-----------------------------------------------------------------
int8_t  menu_pid1(uint8_t argc, const Menu::arg *argv)
{
  if (argc==4){
    pid_1.kP((argv[1].f));    pid_1.kI((argv[2].f));    pid_1.kD((argv[3].f));
    hal.console->printf_P(PSTR("Parsed PID1  : %10.2f %10.2f %10.2f     \n"),pid_1.kP() ,pid_1.kI() , pid_1.kD() );
  }
  return 0;
};
//-----------------------------------------------------------------
int8_t  menu_pid2(uint8_t argc, const Menu::arg *argv)
{
  if (argc==4){
    pid_2.kP((argv[1].f));    pid_2.kI((argv[2].f));    pid_2.kD((argv[3].f));
    hal.console->printf_P(PSTR("Parsed PID2  : %10.2f %10.2f %10.2f     \n"),pid_2.kP() ,pid_2.kI() , pid_2.kD() );
  }
  return 0;
};
//-----------------------------------------------------------------
int8_t  menu_pid3(uint8_t argc, const Menu::arg *argv)
{
  if (argc==4){
    pid_3.kP((argv[1].f));    pid_3.kI((argv[2].f));   pid_3.kD((argv[3].f));
    hal.console->printf_P(PSTR("Parsed PID3  : %10.2f %10.2f %10.2f     \n"),pid_3.kP() ,pid_3.kI() , pid_3.kD() );
  }
  return 0;
};
//-----------------------------------------------------------------
int8_t  menu_pid4(uint8_t argc, const Menu::arg *argv)
{
  if (argc==4){
    pid_4.kP((argv[1].f));   pid_4.kI((argv[2].f));   pid_4.kD((argv[3].f));
    hal.console->printf_P(PSTR("Parsed PID4  : %10.2f %10.2f %10.2f     \n"),pid_4.kP() ,pid_4.kI() , pid_4.kD() );
  }
  return 0;
};

//-------------------------------------------------------------------------------
int8_t  menu_xtrack(uint8_t argc, const Menu::arg *argv)
{
  if (argc==2){
    float  k =argv[1].f;
    k_xtrack = min(max(k,0.0),1.0);
    hal.console->printf_P(PSTR("Parsed xtrack  : %10.3f \n"),k_xtrack );
  }
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_auv_craft(uint8_t argc, const Menu::arg *argv)
{
  kill_mission();
  Solar_craft_setup();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_solar_craft(uint8_t argc, const Menu::arg *argv)
{
  kill_mission();
  Solar_craft_setup();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_kite_craft(uint8_t argc, const Menu::arg *argv)
{
  kill_mission();
  kite_craft_setup();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_plane_craft(uint8_t argc, const Menu::arg *argv)
{
  kill_mission();
  plane_craft_setup();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_erase_logs(uint8_t argc, const Menu::arg *argv)
{
  erase_logs();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_start_cc_mission(uint8_t argc, const Menu::arg *argv)
{
   kill_mission();
   start_CC_mission();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_start_gps_mission(uint8_t argc, const Menu::arg *argv)
{
   kill_mission();
   start_GPS_mission();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_kill_mission(uint8_t argc, const Menu::arg *argv)
{
  kill_mission();
  return 0;
};
//-------------------------------------------------------------------------------
int8_t  menu_read_log(uint8_t argc, const Menu::arg *argv)
{
  uint16_t log_number= (uint16_t)(argv[1].i);  
  hal.console->printf_P(PSTR("Will try to read log #%i :-) \n"),(int)log_number);
 wait_ms(1000);
  if (log_number>0) {flash_read_all_packets(log_number);}
  return 0;
};
//-------------------------------------------------------------------------------
const struct Menu::command top_menu_commands[] PROGMEM = {
    {"a",              menu_print_settings},
    {"pid1",           menu_pid1},
    {"pid2",           menu_pid2},
    {"pid3",           menu_pid3},
    {"pid4",           menu_pid4},
    {"xtrack",         menu_xtrack},
    {"auv",            menu_auv_craft},
    {"solar",          menu_solar_craft},
    {"kite",           menu_kite_craft},
    {"plane",          menu_plane_craft},
    {"eraselogs",      menu_erase_logs},
    {"readlog",        menu_read_log},
    {"c",              menu_start_cc_mission},
    {"g",              menu_start_gps_mission},
    {"q",              menu_kill_mission},

};
//-------------------------------------------------------------------------------
MENU(top, "menu", top_menu_commands);

//-------------------------------------------------------------------------------
static void menu_check_input()
{
    top.run();
}
//-------------------------------------------------------------------------------

// setup menu limits
static void setup_Menu(void)
{
    top.set_limits(64, 6);
}
