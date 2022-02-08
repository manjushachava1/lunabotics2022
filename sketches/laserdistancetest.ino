#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 13); //Define soft serial port, port 13 is TX, port 12 is RX
uint8_t Data[34] = {0};

void setup()
{
  Serial.begin(115200);
  mySerial.begin(115200);
}


void loop()
{

  readData(Data);
  int Distance = readD(Data);
  if (Distance >= 0) {
    Serial.print("Distance: "); Serial.print(Distance); Serial.println(" mm");
    Serial.print("\n");
  } else {
    Serial.println("Invalid data");
    Serial.print("\n");
  }
  delay(500);
}

int readD(uint8_t *buf)
{
  int d;
  char *p = strstr(buf, "Range Valid");
  if (p != 0) {
    d = atoi(&Data[25]);
    return d;
  } else {
    return -1;
  }
}

void readData(uint8_t *buf)
{
  bool flag = 0;
  uint8_t ch;
  while (!flag) {
    if (readN(&ch, 1) == 1) {
      if (ch == 'S') {
        Data[0] = ch;
        if (readN(&ch, 1) == 1) {
          if (ch == 't') {
            Data[1] = ch;
            if (readN(&ch, 1) == 1) {
              if (ch == 'a') {
                Data[2] = ch;
                if (readN(&Data[3], 30) == 30 && Data[31] == 'm' && Data[32] == 'm') {
                  flag = 1;
                }
              }
            }
          }
        }
      }
    }
  }
}
int readN(uint8_t *buf, size_t len)
{
  size_t offset = 0, left = len;
  long curr = millis();
  while (left) {
    if (mySerial.available()) {
      buf[offset++] = mySerial.read();
      left--;
    }
    if (millis() - curr > 500) {
      break;
    }
  }
  return offset;
}
