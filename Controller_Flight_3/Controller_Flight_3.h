
typedef struct _EEPSTRUCT {
  int YawTrim;
  int ThrottleTrim;
  int PitchTrim;
  int RollTrim;
  double Yaw_Kp;
  double Yaw_Ti;
  double Yaw_Td;
  double Throttle_Kp;
  double Throttle_Ti;
  double Throttle_Td;
  double Pich_Kp;
  double Pich_Ti;
  double Pich_Td;
  double Roll_Kp;
  double Roll_Ti;
  double Roll_Td;
} EEPSTRUCT;

#define EEP_START_ADRESS 20
#define RANGE_CHECK(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define LowB(x) (x & 0xFF)
#define HighB(x) (x >> 8)

enum tType
{
  tType_Attitude,
  tType_GyroBias,
  tType_TrimAll,
  tType_IrMessage,
  tType_ImuRawAndAngl,
  tType_Pressure,
  tType_Temperature,
  tType_AnalogStick,
  tType_ControlState
};

enum rType
{
  rType_Command,
  rType_TrimAll,
  rType_Gains
};

enum cmdType
{
  cmdType_Control,
  cmdType_EEP_Write,
  cmdType_EEP_Read,
  cmdType_TrimTune,
  cmdType_TrimSet,
  cmdType_Stop,
  cmdType_Hover,
  cmdType_GainTune,
};

