import  processing.serial.*; //<>// //<>//

ReceiveData r_data;
int[]   data;
Serial  serial;

void setup() {  
  size(400, 250);
  data = new int [width];
  r_data = new ReceiveData();
  serial = new Serial( this, Serial.list()[0], 9600 );
}

public void settings() {
  size(800, 800, P3D);
}

void serialEvent(Serial port) {
  r_data.Receive(port);
}

void draw() {
  background(0);
  fill(0xaa);
  ellipse(width*1/4+r_data.AnalogStick.Yaw, height*3/4-r_data.AnalogStick.Throttle, 20, 20);
  ellipse(width*3/4+r_data.AnalogStick.Roll, height*3/4-r_data.AnalogStick.Pitch, 20, 20);
  
  //PFont font = createFont("Arial", 30);
  //textFont(font);
  
  textSize(30);  
  textAlign(RIGHT, TOP);
  fill(0xaa);
  text("ControlState: "+strs[r_data.ControlState], width-10, 0+10);
  
  translate(width/2, height*1/3);
  rotateZ(radians(r_data.Attitude.Roll));
  rotateX(radians(r_data.Attitude.Pitch));
  rotateY(radians(-r_data.Attitude.Yaw));
  box(200, 5, 300);
}

void keyPressed() {
  if (key == 'c') {
    serial.write('c');
  } else if (key == 's') {
    serial.write('s');
  } else if (key == 't') {
    serial.write('t');
  }
}

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
    tType_ControlState,
};

String[] strs = { 
  "Control", 
  "EEP_Write", 
  "EEP_Read", 
  "TrimTune", 
  "TrimSet", 
  "Stop", 
  "Hover", 
  "GainTune", 
};


public class ReceiveData {

  private static final byte START1 = 0x0A;
  private static final byte START2 = 0x55;
  private static final byte MAX_PACKET_LENGTH = 100;
  private byte cmdBuff[] = new byte[MAX_PACKET_LENGTH];
  private byte dataBuff[] = new byte[MAX_PACKET_LENGTH];
  private byte cmdIndex;
  private byte checkHeader;
  private byte receiveDtype;
  private byte receiveLength;

  Attitude_t Attitude = new Attitude_t();
  GyroBias_t GyroBias = new GyroBias_t();
  TrimAll_t TrimAll = new TrimAll_t();
  IrMessage_t IrMessage = new IrMessage_t();
  ImuRawAndAngl_t ImuRawAndAngl = new ImuRawAndAngl_t();
  Pressure_t Pressure = new Pressure_t();
  Temperature_t Temperature = new Temperature_t();
  AnalogStick_t AnalogStick = new AnalogStick_t(); 
  byte ControlState;


  private int UniteByte(byte upper, byte lower) {
    return ((upper<<8) | (lower & 0xFF) );
  }

  public void Receive(Serial port) {
    if ( port.available() >= 0 ) {
      int input = port.read();
      cmdBuff[cmdIndex++] = (byte)input;

      if (cmdIndex >= MAX_PACKET_LENGTH) {
        checkHeader = 0;
        cmdIndex = 0;
      } else {
        if (cmdIndex == 1) {
          if (cmdBuff[0] == START1) {
            checkHeader = 1;
          } else {
            checkHeader = 0;
            cmdIndex = 0;
          }
        } else if (cmdIndex == 2)
        {
          if (checkHeader == 1) {
            if (cmdBuff[1] == START2) {
              checkHeader = 2;
            } else {
              checkHeader = 0;
              cmdIndex = 0;
            }
          }
        } else if (checkHeader == 2) {
          if (cmdIndex == 3) {
            receiveDtype =  cmdBuff[2];
            dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
          } else if (cmdIndex == 4) {
            receiveLength = cmdBuff[3];
            dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
          } else if (cmdIndex > 4) {
            if (receiveLength + 4 >= cmdIndex) {
              dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
            }
            if (receiveLength + 4 <= cmdIndex) {
              if (receiveDtype == tType.tType_Attitude.ordinal()) {
                Attitude.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                Attitude.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                Attitude.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                Attitude.AliveCnt = dataBuff[8];
                //print(Attitude.Roll); 
                //print(" ");
                //print(Attitude.Pitch); 
                //print(" ");
                //print(Attitude.Yaw); 
                //print(" ");
                //print(Attitude.AliveCnt); 
                //print(" ");
                //println();
              } else if (receiveDtype == tType.tType_GyroBias.ordinal()) {
                /* not available */
                GyroBias.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                GyroBias.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                GyroBias.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                GyroBias.AliveCnt = dataBuff[8];
                //print(GyroBias.Roll); print(" ");
                //print(GyroBias.Pitch); print(" ");
                //print(GyroBias.Yaw); print(" ");
                //print(GyroBias.AliveCnt); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_TrimAll.ordinal()) {
                TrimAll.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                TrimAll.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                TrimAll.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                TrimAll.Throttle = UniteByte(dataBuff[9], dataBuff[8]);
                TrimAll.Wheel = UniteByte(dataBuff[11], dataBuff[10]);
                TrimAll.AliveCnt = dataBuff[12];
                //print(TrimAll.Roll); print(" ");
                //print(TrimAll.Pitch); print(" ");
                //print(TrimAll.Throttle); print(" ");
                //print(TrimAll.Wheel); print(" ");
                //print(TrimAll.AliveCnt); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_IrMessage.ordinal()) {
                /* not available */
                IrMessage.Direction = dataBuff[2];
                IrMessage.IrMessage[0] = dataBuff[3];
                IrMessage.IrMessage[1] = dataBuff[4];
                IrMessage.IrMessage[2] = dataBuff[5];
                IrMessage.IrMessage[3] = dataBuff[6];
                IrMessage.AliveCnt     = dataBuff[7];
                //print(IrMessage.Direction);    print(" ");
                //print(IrMessage.IrMessage[0]); print(" ");
                //print(IrMessage.IrMessage[1]); print(" ");
                //print(IrMessage.IrMessage[2]); print(" ");
                //print(IrMessage.IrMessage[3]); print(" ");
                //print(IrMessage.AliveCnt);     print(" ");
                //println();
              } else if (receiveDtype == tType.tType_ImuRawAndAngl.ordinal()) {
                /* not available */
                ImuRawAndAngl.AccX = UniteByte(dataBuff[3], dataBuff[2]);
                ImuRawAndAngl.AccY = UniteByte(dataBuff[5], dataBuff[4]);
                ImuRawAndAngl.AccZ = UniteByte(dataBuff[7], dataBuff[6]);
                ImuRawAndAngl.GyroRoll = UniteByte(dataBuff[9], dataBuff[8]);
                ImuRawAndAngl.GyroPitch = UniteByte(dataBuff[11], dataBuff[10]);
                ImuRawAndAngl.GyroRoll = UniteByte(dataBuff[13], dataBuff[12]);
                ImuRawAndAngl.AngleRoll = UniteByte(dataBuff[15], dataBuff[14]);
                ImuRawAndAngl.AnglePitch = UniteByte(dataBuff[17], dataBuff[16]);
                ImuRawAndAngl.AngleRoll = UniteByte(dataBuff[19], dataBuff[18]);
                ImuRawAndAngl.AliveCnt = dataBuff[20];
                //print(ImuRawAndAngl.AccX); print(" ");
                //print(ImuRawAndAngl.AccY); print(" ");
                //print(ImuRawAndAngl.AccZ); print(" ");
                //print(ImuRawAndAngl.GyroRoll); print(" ");
                //print(ImuRawAndAngl.GyroPitch); print(" ");
                //print(ImuRawAndAngl.GyroRoll); print(" ");
                //print(ImuRawAndAngl.AngleRoll); print(" ");
                //print(ImuRawAndAngl.AnglePitch); print(" ");
                //print(ImuRawAndAngl.AngleRoll); print(" ");
                //print(ImuRawAndAngl.AliveCnt); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_Pressure.ordinal()) {
                /* not available */
                for (int i=0; i<16; i++) {
                  Pressure.Pressure[i] = dataBuff[i+2];
                }
                Pressure.AliveCnt = dataBuff[18];
                //for (int i=0; i<16; i++) {
                //  print(Pressure.Pressure[i]); print(" ");
                //}
                //print(Pressure.AliveCnt);
                //println();
              } else if (receiveDtype == tType.tType_Temperature.ordinal()) {
                /* not available */
                for (int i=0; i<8; i++) {
                  Temperature.Temperature[i] = dataBuff[i+2];
                }
                Temperature.AliveCnt = dataBuff[10];
                //for (int i=0; i<8; i++) {
                //  print(Temperature.Temperature[i]); print(" ");
                //}
                //print(Temperature.AliveCnt); 
                //println();
              } else if (receiveDtype == tType.tType_AnalogStick.ordinal()) {
                AnalogStick.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                AnalogStick.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                AnalogStick.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                AnalogStick.Throttle = UniteByte(dataBuff[9], dataBuff[8]);
                //print(AnalogStick.Roll); print(" ");
                //print(AnalogStick.Pitch); print(" ");
                //print(AnalogStick.Yaw); print(" ");
                //print(AnalogStick.Throttle); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_ControlState.ordinal()) {
                ControlState = dataBuff[2];
                //println(strs[ControlState]);
              }
              checkHeader = 0;
              cmdIndex = 0;
            }
          }
        } else {
          checkHeader = 0;
          cmdIndex = 0;
        }
      }
    }
  }
}



class Attitude_t {
  int Roll;
  int Pitch;
  int Yaw;
  byte AliveCnt;
}

class GyroBias_t {
  int Roll;
  int Pitch;
  int Yaw;
  byte AliveCnt;
}

class TrimAll_t {
  int Roll;
  int Pitch;
  int Yaw;
  int Throttle;
  int Wheel;
  byte AliveCnt;
}

class IrMessage_t {
  int Direction;
  int IrMessage[] = new int[4];
  byte AliveCnt;
}

class ImuRawAndAngl_t {
  int AccX;
  int AccY;
  int AccZ;
  int GyroRoll;
  int GyroPitch;
  int GyroYaw;
  int AngleRoll;
  int AnglePitch;
  int AngleYaw;
  byte AliveCnt;
}

class Pressure_t {
  int Pressure[] = new int[16];
  byte AliveCnt;
}

class Temperature_t {
  int Temperature[] = new int[8];
  byte AliveCnt;
}

class AnalogStick_t {
  int Roll;
  int Pitch;
  int Yaw;
  int Throttle;
}