/*
RC Receiver Header File
Author:    Omer Ikram ul Haq
Date:      2014-05-18
Location:  Pakistan
Version:   ver 0.1 beta
Test with: Arduino Mega and Devention rx701
Modified for: Dual Mode Controller
*/

#ifndef rc_rx_h
  #define rc_rx_h
  
  extern float CH1,CH2,CH3,CH4;
  
  void init_rc_rx();
  void read_rc_rx();  
#endif
