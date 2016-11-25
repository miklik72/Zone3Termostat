#include "Arduino.h"
#include "DFR_KeyMM.h"

static int DEFAULT_KEY_PIN = 0;
static int DEFAULT_THRESHOLD = 5;         // treshold for analog pin

static int UPKEY_ARV = 100; //that's read "analogue read value"
static int DOWNKEY_ARV = 255;
static int LEFTKEY_ARV = 407;
static int RIGHTKEY_ARV = 0;
static int SELKEY_ARV = 638;
static int NOKEY_ARV = 1023;

DFR_KeyMM::DFR_KeyMM()
{
  _refreshRate = 50;     // delay bitween two measurements
  _keyPin = DEFAULT_KEY_PIN;
  _threshold = DEFAULT_THRESHOLD;
  _keyIn = NO_KEY;
  _curInput = NO_KEY;
  _curKey = NO_KEY;
  _prevInput = NO_KEY;
  _prevKey = NO_KEY;
  _oldTime = 0;
}

int DFR_KeyMM::getKey()
{
 if (millis() > _oldTime + _refreshRate)  // only if oldtime + refresh rate is higher then current time
 {
    _prevInput = _curInput;               // save old button state
    _curInput = analogRead(_keyPin);      // read new state
    // ???????
    _threshold = DEFAULT_THRESHOLD;
    
    //if (_curInput == _prevInput)          
    if (abs(_curInput - _prevInput) < _threshold)          //button input wasn't changed or only few
    {
      _change = false;
      _curKey = _prevKey;
    }
    else                                  // button state was chenged
    {
      _change = true;
      _prevKey = _curKey;

      if (_curInput > UPKEY_ARV - _threshold && _curInput < UPKEY_ARV + _threshold ) _curKey = UP_KEY;             // return button code
      else if (_curInput > DOWNKEY_ARV - _threshold && _curInput < DOWNKEY_ARV + _threshold ) _curKey = DOWN_KEY;
      else if (_curInput > RIGHTKEY_ARV - _threshold && _curInput < RIGHTKEY_ARV + _threshold ) _curKey = RIGHT_KEY;
      else if (_curInput > LEFTKEY_ARV - _threshold && _curInput < LEFTKEY_ARV + _threshold ) _curKey = LEFT_KEY;
      else if (_curInput > SELKEY_ARV - _threshold && _curInput < SELKEY_ARV + _threshold ) _curKey = SELECT_KEY;
      else _curKey = NO_KEY;
   }

   if (_change) return _curKey; else return SAMPLE_WAIT;
   _oldTime = millis();
 }
   else
  {
    return SAMPLE_WAIT;
  }
}

int DFR_KeyMM::getValue()
{
    return analogRead(_keyPin);
}

void DFR_KeyMM::setRate(int rate)
{
  _refreshRate = rate;
}
