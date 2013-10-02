// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//---------------------------------------------------------------------------
// Jakob Kuttenkeuler, jakob@kth.se
//---------------------------------------------------------------------------

void init_analog(){
  hal.console->printf_P(PSTR("  Analog:  "));
  Ch0 = hal.analogin->channel(0);
  Ch1 = hal.analogin->channel(1);
  Ch2 = hal.analogin->channel(2);
  Ch3 = hal.analogin->channel(3);
  Ch4 = hal.analogin->channel(4);
  Vcc = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
  hal.console->printf_P(PSTR(" ok. \n"));
}
//-------------------------------------------------------------------------------

void read_analogue_channels(){
   if (abs(time_ms-last_adc_update)>20){
      last_adc_update = time_ms;
      adc0 =  Ch0->voltage_average();
      adc1 =  Ch1->voltage_average();
      adc2 =  Ch2->voltage_average();
      adc3 =  Ch3->voltage_average();
      adc4 =  Ch4->voltage_average();
      vcc  =  Vcc->voltage_average();
      //hal.console->printf_P(PSTR("Analogue channels:  adc0=%.2f, adc1=%.2f,   adc2=%.2f, vcc: %f\r\n"),adc0,adc1, adc1, vcc);
    }
}//-------------------------------------------------------------------------------









