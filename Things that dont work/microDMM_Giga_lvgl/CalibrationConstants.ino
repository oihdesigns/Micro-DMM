/* Unused for Giga
void eepromSetup(){
    if(EEPROM.read(1) == 1){ //Read the EEPROM, if address 1 is 1 then use these coefficients
      CF_A = 1;
      CF_B = 0.983;
      CF_C = 0.9926;
      CF_D = 0.9735;
      CF_E = 1.0732;
      CF_F = 1.0281;
      CF_G = 1.0018;
      CF_H = 0.9995;
      CF_I = 0.9987;
      CF_J = 1.0054;
      CF_K = 0.9975;
      CF_L = 0.9904;
      CF_M = 0.9883;
      CF_N = 0.8741;
      CF_O = 0.7294;
      VOLTAGE_SCALE = 63.539; // Calibration scale factor for voltage input{
  } else if(EEPROM.read(1) == 2){
      CF_A = 1.0528;
      CF_B = 1.0135;
      CF_C = 1.0063;
      CF_D = 0.9853;
      CF_E = 0.9695;
      CF_F = 1.0183;
      CF_G = 0.9998;
      CF_H = 1.0002;
      CF_I = 0.9984;
      CF_J = 1.0056;
      CF_K = 0.9999;
      CF_L = 0.996;
      CF_M = 1.0158;
      CF_N = 0.927;
      CF_O = 0.6924;
      VOLTAGE_SCALE = 46.392; // 20250527
  } else if(EEPROM.read(1) == 3){
      CF_A = 0.9867;
      CF_B = 1.0007;
      CF_C = 0.9987;
      CF_D = 0.9971;
      CF_E = 0.9952;
      CF_F = 0.9961;
      CF_G = 0.997;
      CF_H = 1.0044;
      CF_I = 1.0031;
      CF_J = 1.0023;
      CF_K = 1.0044;
      CF_L = 1.0092;
      CF_M = 1.0218;
      CF_N = 1.0725;
      CF_O = 1.2269;
      EEPROM_MAXV = 4.994;
      EEPROM_SleepV = 0.615;
      constantI = 0.020087;
      VOLTAGE_SCALE = (46.4564*2); // 20250619
  } else if(EEPROM.read(1) == 4){
      CF_A = 0.9828;
      CF_B = 0.9958;
      CF_C = 0.9999;
      CF_D = 0.9979;
      CF_E = 0.9954;
      CF_F = 0.9965;
      CF_G = 0.998;
      CF_H = 1.0034;
      CF_I = 1.0021;
      CF_J = 1.0012;
      CF_K = 1.0031;
      CF_L = 1.0073;
      CF_M = 1.0176;
      CF_N = 1.0611;
      CF_O = 1.113;
      constantI = 0.020073;
      EEPROM_SleepV = 0.612;
      EEPROM_MAXV = 4.998;
      VOLTAGE_SCALE = 46.46764969; // 20250611
  } else if(EEPROM.read(1) == 5){
        CF_A = 1.0;
        CF_B = 1.0;
        CF_C = 1.0;
        CF_D = 1.0;
        CF_E = 1.0;
        CF_F = 1.0;
        CF_G = 1.0;
        CF_H = 1.0;
        CF_I = 1.0;
        CF_J = 1.0;
        CF_K = 1.0;
        CF_L = 1.0;
        CF_M = 1.0;
        CF_N = 1.0;
        CF_O = 1.0;
        VOLTAGE_SCALE = 46.4815; // 
        constantI = 0.020024;
        EEPROM_SleepV = 0.606;
        EEPROM_MAXV = 4.998;
  } else if(EEPROM.read(1) == 6){
      CF_A = 1.0;
      CF_B = 1.0;
      CF_C = 1.0;
      CF_D = 1.0;
      CF_E = 1.0;
      CF_F = 1.0;
      CF_G = 1.0;
      CF_H = 1.0;
      CF_I = 1.0;
      CF_J = 1.0;
      CF_K = 1.0;
      CF_L = 1.0;
      CF_M = 1.0;
      CF_N = 1.0;
      CF_O = 1.0;
      VOLTAGE_SCALE = 46.41618; // 
      constantI = 0.020062;
      EEPROM_SleepV = 0.634;
      EEPROM_MAXV = 5.006;
  }else{ //If EEPROM addr 1 is not 1 use these
    // Calibration correction factors for resistance (piecewise)
    CF_A = 1.0;
    CF_B = 1.0;
    CF_C = 1.0;
    CF_D = 1.0;
    CF_E = 1.0;
    CF_F = 1.0;
    CF_G = 1.0;
    CF_H = 1.0;
    CF_I = 1.0;
    CF_J = 1.0;
    CF_K = 1.0;
    CF_L = 1.0;
    CF_M = 1.0;
    CF_N = 1.0;
    CF_O = 1.0;
    VOLTAGE_SCALE = 46.467649; //previous default: 63.5; // Calibration scale factor for voltage input
  }
}
*/