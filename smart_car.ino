#include <Audio.h>
#include <SPI.h>
#include <SDHCI.h>
#include <stdio.h>
#include <BMI160Gen.h>
#include "Adafruit_ILI9341.h"
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF
// For the Adafruit shield, these are the default.
#define TFT_RST 18
#define TFT_DC  25
#define TFT_CS  -1
// the setup function runs once when you press reset or power the board
//A2,A3,A4,A5
#define L2 A2
#define L1 A3
#define R1 A4
#define R2 A5
#define SideL A1
#define SideR A0
int Echo = 12;  // Echo回声脚(P2.0)
int Trig = 11;  //  Trig 触发脚(P2.1)
int Distance = 15;
int SL;    //左循迹红外传感器状态
int SR;    //右循迹红外传感器状态

int Left_motor_back = 9;       //左电机后退(IN1)
int Left_motor_go = 5;         //左电机前进(IN2)

int Right_motor_go = 6;        // 右电机前进(IN3)
int Right_motor_back = 3;    // 右电机后退(IN4)

int Right_motor_en = 8;      // 右电机前进(EN2)
int Left_motor_en = 7;      // 右电机后退(EN1)

int key = 4;                //定义按键 数字4 接口

float Kp = 14, Ki = 0.2, Kd = 40;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
static int initial_motor_speed = 120; //期望速度
int left_motor_speed = 0;
int right_motor_speed = 0;
uint8_t irs = 0;
uint8_t sideirs = 0;
bool ErrEnd = false;
/*
  place shows the area where the car in
  place == 0 -> in line able place
  place == 1 -> in line blank place
  place == 2 -> in hill
*/
int place = 0;
int place_count = 0;
int beep_count = 0;
int beep_diao = 1;
int music_count = 0;
int tft_count = 0;
bool pass = false;
bool playing = false;
bool line = false;
bool isbeep_init = false;
bool isbeep_stop = false;
bool ON_beep = false;
SDClass theSD;
AudioClass *theAudio;
File myFile;
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

static void audio_attention_cb(const ErrorAttentionParam *atprm)
{
  puts("Attention!");

  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING)
  {
    ErrEnd = true;
  }
}


class Score
{
  public:

    typedef struct {
      int fs;
      int time;
    } Note;

    void init() {
      pos = 0;
    }

    Note get(int sound) {
      return data[sound];
    }

  private:

    int pos;

    Note data[9] =
    {
      {262, 500},
      {294, 500},
      {330, 500},
      {349, 500},
      {349, 500},
      {330, 500},
      {294, 500},
      {262, 500},
      {0, 0}
    };
};

Score theScore;


void setup() {
  Serial.begin(115200);
  pinMode(Left_motor_go, OUTPUT); // PIN 5 (PWM)
  pinMode(Left_motor_back, OUTPUT); // PIN 9 (PWM)
  pinMode(Right_motor_go, OUTPUT);// PIN 6(PWM)
  pinMode(Right_motor_back, OUTPUT);// PIN 3 (PWM)
  pinMode(key, INPUT);//定义按键接口为输入接口
  digitalWrite(key, HIGH);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(BLACK);
  tft.setTextColor(GREEN, BLACK);
  tft.setTextSize(3);
  diplay_now("Loading");
  initMusic();
  BMI160.begin();
  BMI160.setAccelerometerRange(2);
  pinMode(Echo, INPUT);    // 定义超声波输入脚
  pinMode(Trig, OUTPUT);   // 定义超声波输出脚
  diplay_now("  START");
  tft.setCursor(0, 80);
  tft.setTextSize(3);
  tft.print(" Press button");
  keysacn();
  tft.setCursor(0, 80);
  tft.print("                            ");
  diplay_now(" Ruing");
}



void Distance_test()   // 量出前方距离
{
  digitalWrite(Trig, LOW);   // 给触发脚低电平2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
  delayMicroseconds(10);//10us
  digitalWrite(Trig, LOW);    // 持续给触发脚低电
  float Fdistance = pulseIn(Echo, HIGH);  // 读取高电平时间(单位：微秒)
  Fdistance = Fdistance / 58;       //为什么除以58等于厘米，  Y米=（X秒*344）/2
  // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
  Serial.print("Distance:");      //输出距离（单位：厘米）
  Serial.println(Fdistance);         //显示距离
  Distance = Fdistance;
}


void read_ir_values()
{
  int temp[4];
  temp[0] = analogRead(L2);
  temp[1] = analogRead(L1);
  temp[2] = analogRead(R1);
  temp[3] = analogRead(R2);

  int side[2];
  side[0] = analogRead(SideR);
  side[1] = analogRead(SideL);

  for (int i = 0; i < 4; i++) {
    if (temp[i] < 300) {
      bitSet(irs, (i));
    }
    else {
      bitClear(irs, (i));
    }
  }

  for (int i = 0; i < 2; i++)
  {
    if (side[i] < 200)
    {
      bitSet(sideirs, (i));
    }
    else
    {
      bitClear(sideirs, (i));
    }
  }

  if (place == 0)
  {
    switch (irs) {
      case B0000:
        place_count++;
        //beep_count++;
        if (isbeep_init == true && analogRead(A0) > 200 && analogRead(A1) > 200)
        {
          NowBeep(beep_diao++);
        }
        if (place_count > 8)
        {
          /*if (beep_count > 15)
            {
            beep_count = 0;
            }*/
          initial_motor_speed = 85;
          switch (sideirs)
          {
            case B01:
              error = 2.5;
              break;
            case B10:
              error = -2.5;
              break;
            default:
              break;
              place_count = 0;
          }
        }
        //initial_motor_speed = 100;
        break;
      case B1111:
        switch (playing)
        {
          case false:
            theAudio->startPlayer(AudioClass::Player0);
            playing = true;
            pass = false;
            break;

          case true:
            //theAudio->stopPlayer(AudioClass::Player0);
            //myFile.close();
            if (music_count > 300)
            {
              playing = false;
              pass = true;
            }
            break;
        }
        break;
      case B0001: error = 4; StopBeep(); place = 0; break;
      case B0011: error = 2.5; initial_motor_speed = 120; StopBeep(); place = 0; break;
      case B0010: error = 1; initial_motor_speed = 150; StopBeep(); place = 0; break;
      case B0110: error = 0; initial_motor_speed = 200;  StopBeep(); place = 0; break;
      case B0100: error = -1; initial_motor_speed = 150; StopBeep(); place = 0; break;
      case B1100: error = -2.5; initial_motor_speed = 120; StopBeep(); place = 0; break;
      case B1000: error = -4; StopBeep(); place = 0; break;
    }
  }





}

void calculate_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  PID_value = constrain(PID_value, -100, 100);

  previous_error = error;
}

void motor_control()
{
  //计算每个电机的速度
  left_motor_speed = initial_motor_speed - PID_value;
  right_motor_speed = initial_motor_speed + PID_value;
  if (place_count > 8)
  {
    constrain(left_motor_speed, 20, 100);
    constrain(right_motor_speed, 20, 100);
  }
  else
  {
    constrain(left_motor_speed, 80, 200);
    constrain(right_motor_speed, 80, 200);
  }
  LeftRoll(left_motor_speed);
  RightRoll(right_motor_speed);
}

void LeftRoll(int speed)
{
  if (speed > 0)
  {
    digitalWrite(Left_motor_back, LOW);  // 左电机前进
    digitalWrite(Left_motor_go, HIGH);
    analogWrite(Left_motor_go, speed);//PWM比例0~255调速，左右轮差异略增减
  }
  else
  {
    digitalWrite(Left_motor_go, LOW);
    digitalWrite(Left_motor_back, HIGH);
    analogWrite(Left_motor_back, -speed);
  }
}

void RightRoll(int speed)
{
  if (speed > 0)
  {
    digitalWrite(Right_motor_go, HIGH);  // 右电机前进
    digitalWrite(Right_motor_back, LOW);
    analogWrite(Right_motor_go, speed);//PWM比例0~255调速，左右轮差异略增减
  }
  else
  {
    digitalWrite(Right_motor_go, LOW);
    digitalWrite(Right_motor_back, HIGH);
    analogWrite(Right_motor_back, -speed);
  }
}

void brake()
{
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  //delay(time * 100);//执行时间，可以调整
}

void keysacn()//按键扫描
{
  int val;




  val = digitalRead(key);//读取数字3 口电平值赋给val
  while (digitalRead(key))//当按键没被按下时，一直循环
  {
    val = digitalRead(key);//此句可省略，可让循环跑空
  }
  while (!digitalRead(key))//当按键被按下时
  {
    delay(10);  //延时10ms
    val = digitalRead(key);//读取数字3 口电平值赋给val
  }
}

void print_debug()
{
  // 打印串口调试信息
  Serial.print("IRS:");
  String irs2bin = String(irs, 2);
  int len = irs2bin.length();
  if (len < 5) {
    for (int i = 0; i < 5 - len; i++) {
      irs2bin += "0" + irs2bin;
    }
  }
  Serial.print(irs2bin);
  Serial.print("   ML:");
  Serial.print(left_motor_speed, OCT);
  Serial.print(" MR:");
  Serial.print(right_motor_speed, OCT);
  Serial.print(" er:");
  Serial.print(error, OCT);
  //  Serial.print(" P:");
  //  Serial.print(Kp, OCT);
  Serial.print(" PID:");
  Serial.print(PID_value, OCT);
  Serial.println();
}

void NowBeep(int sound)
{
  if (ON_beep == true)
  {
    return;
  }
  Score::Note theNote = theScore.get(sound);
  theAudio->setBeep(1, -40, theNote.fs);
}

void climb_beep(int sound)
{
  Score::Note theNote = theScore.get(sound);
  theAudio->setBeep(1, -40, theNote.fs);
}

void StopBeep()
{
  theAudio->setBeep(0, 0, 0);
}

void display_debug()
{
  tft.setTextSize(2);
  tft.setTextColor(GREEN, BLACK);
  tft.setCursor(0, 0);
  tft.print(analogRead(A2)); tft.print(",");
  tft.print(analogRead(A3)); tft.print(",");
  tft.print(analogRead(A4)); tft.print(",");
  tft.print(analogRead(A5)); tft.print(",");
  tft.setCursor(0, 20);
  switch (irs)
  {
    case B0000: tft.print("  -  -  -  -       "); break;
    case B0001: tft.print(" @  -  -  -       "); break;
    case B0011: tft.print(" @ @ -  -       "); break;
    case B0010: tft.print("  -  @  -   -     "); break;
    case B0110: tft.print("  -  @  @  -    "); break;
    case B0100: tft.print("  -   -   @   -    "); break;
    case B1100: tft.print("  -   -   @  @  "); break;
    case B1000: tft.print("  -   -    -  @   "); break;
  }
}


void initMusic()
{
  // start audio system
  theAudio = AudioClass::getInstance();

  theAudio->begin(audio_attention_cb);

  puts("initialization Audio Library");

  /* Set clock mode to normal */
  theAudio->setRenderingClockMode(AS_CLKMODE_NORMAL);

  /* Set output device to speaker with first argument.
     If you want to change the output device to I2S,
     specify "AS_SETPLAYER_OUTPUTDEVICE_I2SOUTPUT" as an argument.
     Set speaker driver mode to LineOut with second argument.
     If you want to change the speaker driver mode to other,
     specify "AS_SP_DRV_MODE_1DRIVER" or "AS_SP_DRV_MODE_2DRIVER" or "AS_SP_DRV_MODE_4DRIVER"
     as an argument.
  */
  theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, AS_SP_DRV_MODE_LINEOUT);

  /*
     Set main player to decode stereo mp3. Stream sample rate is set to "auto detect"
     Search for MP3 decoder in "/mnt/sd0/BIN" directory
  */
  //  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/spif/BIN", AS_SAMPLINGRATE_AUTO, AS_CHANNEL_STEREO);
  err_t err = theAudio->initPlayer(AudioClass::Player0, AS_CODECTYPE_MP3, "/mnt/spif/BIN", 44100, AS_CHANNEL_STEREO);

  /* Verify player initialize */
  if (err != AUDIOLIB_ECODE_OK)
  {
    printf("Player0 initialize error\n");
    exit(1);
  }

  /* Open file placed on SD card */
  myFile = theSD.open("hop.mp3");

  /* Verify file open */
  if (!myFile)
  {
    printf("File open error\n");
    exit(1);
  }
  printf("Open! %d\n", myFile);

  /* Send first frames to be decoded */
  err = theAudio->writeFrames(AudioClass::Player0, myFile);

  if ((err != AUDIOLIB_ECODE_OK) && (err != AUDIOLIB_ECODE_FILEEND))
  {
    printf("File Read Error! =%d\n", err);
    myFile.close();
    exit(1);
  }
  theAudio->setVolume(-160);
}

void initBeep()
{
  theAudio = AudioClass::getInstance();
  theAudio->begin();
  theAudio->setPlayerMode(AS_SETPLAYER_OUTPUTDEVICE_SPHP, 0, 0);
  theScore.init();
}


void diplay_now(String data)
{
  tft.setCursor(60, 20);
  tft.print("Hop II");
  tft.setCursor(0, 120);
  tft.print("                                             ");
  tft.setCursor(50, 120);
  tft.print(data);
}

void loop() {
  music_count++;
  //Serial.print(analogRead(A0)); Serial.print(",");
  //Serial.print(analogRead(A1)); Serial.print(",");
  Serial.print(analogRead(A2)); Serial.print(",");
  Serial.print(analogRead(A3)); Serial.print(",");
  Serial.print(analogRead(A4)); Serial.print(",");
  Serial.print(analogRead(A5)); Serial.println(",");
  //Serial.print(irs, BIN); Serial.println("");
  //display_debug();
  read_ir_values();
  calculate_pid();
  motor_control();
  if (music_count > 230)
  {
    playing = false;
    pass = true;
  }
  if (music_count > 800 && isbeep_init == false)
  {
    initBeep();
    isbeep_init = true;
  }

  if (music_count > 880)
  {
    StopBeep();
    ON_beep = true;
  }
  if (music_count == 880)
  {
    beep_diao = 0;
  }

  if (music_count > 920)
  {
    Distance_test();
    while (Distance < 14)
    {
      brake();
    }
  }
  float ax, ay, az;
  BMI160.readAccelerometerScaled(ax, ay, az);
  if (az < 0.75 && music_count > 780)
  {
    beep_diao++;
    if (beep_diao == 8)beep_diao = 0;
    climb_beep(beep_diao);
  }
  if (pass == false && playing == true)theAudio->writeFrames(AudioClass::Player0, myFile);
  //print_debug();
}
