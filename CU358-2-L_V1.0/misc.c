#include "stm8s_conf.h"
#include "hal_delay.h"
#include "misc.h"
#include "rf-protocol.h"



/*


bool DeBounceKey(void)
{

  static uint8_t KeyRepeat = 0;
  bool KeyValue  = FALSE;
  static bool oldKeyValue = FALSE;
  static bool activeKey = FALSE;
  if(GPIO_ReadInputPin(NAS_PORT, NAS_PIN))
    KeyValue = FALSE;
  else
    KeyValue = TRUE;
  
  if(oldKeyValue == KeyValue)  
  {
    if(KeyRepeat < 20)
      KeyRepeat ++;
  }
  else
    KeyRepeat = 0;
  
  
  oldKeyValue = KeyValue;
  
  if(KeyRepeat >= 5)
    activeKey = KeyValue;
  
  return activeKey;
}

void Nas_10ms_Appl(void)
{
  bool NasKey;
  static bool lastNasKey = FALSE;
  static uint8_t NasKeyRepeat = 0;
  static uint8_t lastTeachMode = 0;
  
  
  NasKey = DeBounceKey();
  
  if(NasKey == lastNasKey)
  {
    if(NasKeyRepeat < 200)
      NasKeyRepeat ++;
  }
  else
  {
    if(NasKey != FALSE && NasKeyRepeat < 100)
    {
      if(getTeachMode())
      {
        clrTeachMode();
      }
      else
      {
        setTeachMode();
      }
      NasKeyRepeat = 0;
    }
    else if(NasKey == FALSE)
    {
      if(getTeachMode() != lastTeachMode)
      {
        NasKeyRepeat = 100;
        lastTeachMode = getTeachMode();
      }
      else
        NasKeyRepeat = 0;
    }
    else
      NasKeyRepeat = 0;
  }
  lastNasKey = NasKey;
}*/