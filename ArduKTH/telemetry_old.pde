// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Here is a shange

//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se

char read_one_char_from_serial(){
  return 'x';
}
//-------------------------------------------------------------------------------
bool shoul_be_this_char_at_serial(char x){   
    return false;
}

//-------------------------------------------------------------------------------
double get_a_float_field_from_serial(){  
   
    return 99.99;
}


//-------------------------------------------------------------------------------
void parse_deviation_mission()
{
   // Format :  #d,pwm,d
   // Example:  #d,1600,d
   hal.console->printf_P(PSTR("Parsing Deviation mission: "));
   bool flag =true; // The flag sayin "ok" or "not ok"
   if (!shoul_be_this_char_at_serial(',')){flag =false;}
   int rpm_pwm = (int)get_a_float_field_from_serial(); 
   if (rpm_pwm<1000 || rpm_pwm>2000){flag=false;} 
   if (!shoul_be_this_char_at_serial('d')){flag =false;}
   if (flag==false){hal.console->printf_P(PSTR("   ... Sorry, parsing failed. \n")); return;}
  
   kill_mission();
   //hal.console->printf_P(PSTR("You wanna do Deviation, lets try :-)\n"));
   //start_Deviation_mission();
   mission_start_pos = current_pos;
   hal.console->printf_P(PSTR("\nStarting Deviation mission (at position %.5f,  %.5f)  \n"),ToDeg(mission_start_pos.lon), ToDeg(mission_start_pos.lat));
   arm_RC();
   target_rpm        = rpm_pwm;
   mission_start_ms  = time_ms; 
   mission_ms        = 0;
   current_leg_nr    = 0;
   ctrl_mode         = 'd';

   hal.console->printf_P(PSTR("Flirting with motor control..."));
   hal.rcout->write(CH_3,1500); // To make Motor-control happy
   hal.scheduler->delay(2000);  // Wait for Motor-control
   hal.console->printf_P(PSTR(" done. \n"));   

}
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
void parse_CC_mission(){
  // Example   #C,  Nlegs  ,  duration,    course,     depth,       rpm,  duration,    course,     depth,       rpm,C
  //           #C,   2     ,    20    ,     123.4,       1  ,       1600,       20,       180,         2,       200,C
  // Kvadrat:  #C,4,    10, 0,0,0,  10,90,0,0,  10,180,0,0,    10,270,0,0,C
  kill_mission();
  hal.console->print("Parsing CC mission: ");
  bool flag = true; // The flag saying "ok" or "not ok"
  if (!shoul_be_this_char_at_serial(',')){flag =false;}
  Nlegs_cc = (int)get_a_float_field_from_serial();  // Number of legs
  for (int ileg=0; ileg<Nlegs_cc ; ileg++){ 
     CC_mission[ileg].duration = (float)get_a_float_field_from_serial();
     CC_mission[ileg].course   = (float)ToRad((float)get_a_float_field_from_serial());
     CC_mission[ileg].depth    = (float)get_a_float_field_from_serial();
     CC_mission[ileg].rpm      = (float)get_a_float_field_from_serial();
     if (CC_mission[ileg].duration==NAN || CC_mission[ileg].course==NAN || CC_mission[ileg].depth==NAN  || CC_mission[ileg].rpm==NAN ) {flag =false;} 
   }
   if (!shoul_be_this_char_at_serial('C')){flag =false;} // Identifier check

   if (flag==true){
      //mission = prospect_mission;
      //for (int ii=0; ii<Nlegs; ii++) {mission[ii] = mission[ii]; }
       hal.console->printf_P(PSTR("Nlegs = %i\n"),Nlegs_cc);
      hal.console->print("Ok\n");   
      print_CC_mission();
   }
   else{
      hal.console->print("FAIL => I will therefor setup default mission.\n");
      setup_default_CC_mission();
   } 
}
//-------------------------------------------------------------------------------
static void parse_GPS_mission(){
  //          #G,Nlegs,  lon,lat,depth,wp_radius,rpm,     lon,lat,depth,wp_radiusrpm,G
  // Example  #G,3, 18.07205,59.34837,0,20,1600, 18.26582,59.31257,0,20,1700, 18.07205,59.34837,0,20,1500,G
  //
  kill_mission();
  hal.console->print("Parsing GPS-mission: ");
  bool flag = true; // The flag saying "ok" or "not ok"
  if (!shoul_be_this_char_at_serial(',')){flag =false;}
  Nlegs_GPS = (int)get_a_float_field_from_serial();  // Number of legs
  
  for (int ileg=0; ileg<Nlegs_GPS ; ileg++){ 
     GPS_mission[ileg].lon            = (float)ToRad((float)get_a_float_field_from_serial());
     GPS_mission[ileg].lat            = (float)ToRad((float)get_a_float_field_from_serial());
     GPS_mission[ileg].depth          = (float)get_a_float_field_from_serial();
     GPS_mission[ileg].wp_radius      = (float)get_a_float_field_from_serial();
     GPS_mission[ileg].rpm            = (float)get_a_float_field_from_serial();
   }
   if (!shoul_be_this_char_at_serial('G')){flag =false;} // Identifier check
 
   if (flag==true){
    //mission = prospect_mission;
    //for (int ii=0; ii<Nlegs; ii++) {mission[ii] = mission[ii]; }
       hal.console->printf_P(PSTR("Nlegs = %i\n"),Nlegs_GPS);
    hal.console->print("Ok\n");   
    print_GPS_mission();
   }
   else{
    hal.console->print("FAIL => I will therefor setup default mission.\n");
    setup_default_GPS_mission();
   }
}


//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
void parse_incoming_telemetry()  {
   char Id = read_one_char_from_serial();
   switch ( Id ) {
     //case '1': parse_PID_1();                break;
     //case '2': parse_PID_2();                break;
     //case '3': parse_PID_3();                break;
     //case '4': parse_PID_4();                break;
     //case 'a': print_settings();             break;
     //case 'x': parse_k_Xtrack();             break;
     //case 'S': kill_mission();Solar_craft_setup();          break;
     case 'E': kill_mission();erase_logs();                 break;
     //case 'A': kill_mission();AUV_craft_setup();            break;
     //case 'P': kill_mission();Plane_craft_setup();          break;
     //case 'K': kill_mission();Kite_craft_setup();           break;
     //case 'c': kill_mission();start_CC_mission();           break;
     //case 'g': kill_mission();start_GPS_mission();          break;
     case 'C': kill_mission();parse_CC_mission();           break;
     case 'G': kill_mission();parse_GPS_mission();          break;
     case 'd': kill_mission();parse_deviation_mission();       break;
     case 'r': kill_mission(); ctrl_mode='r';  hal.console->printf_P(PSTR("\nRC dude\n"));break;
     case 't': continously_send=true; break;
     //case 'q': kill_mission();break;
     //case 'X': parse_crash_limits();break;
     //case 's': kill_mission();continously_send=false;parse_read_logs();break;
     default :  ;                     break;
   } 
   
}


//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

