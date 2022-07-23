#include <TimerOne.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

/*------------------------------KeyPad------------------------------*/
char sophong1;
char sophong2;
String sotangstr;
String sophongstr;
int sophongint,sotangint;
byte letter;

const byte ROWS = 4;
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = 
{
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {34,36,38,40};
byte colPins[COLS] = {42,44,46,48};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
/*-------------------------------LCD--------------------------------*/
LiquidCrystal_I2C lcd(0x27,16,4);
/*-----------------------------RFID---------------------------------*/
#define RST_PIN         7         
#define SS_PIN          53         
MFRC522 mfrc522(SS_PIN, RST_PIN);  
/*------------------ Khai báo MPU6050 ------------------------------*/
MPU6050 mpu;
unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0, yaw_t = 0;
/*------------------ Khai báo biến PID ---------------------------*/
double T, xung_L, xung_R, xung;
double tocdo_L, Tocdodat_L, tocdo_R, Tocdodat_R;
double E_L, E1_L, E2_L, E_R, E1_R, E2_R;
double alpha, beta, gamma, Kp, Kd, Ki;
double Output_L, LastOutput_L, Output_R, LastOutput_R;
/*----------------- Khai báo biến HC - SR04 ----------------------*/
const int trig = 23;
const int echo = 22;
unsigned long duration; // biến đo thời gian
int distance;           // biến lưu khoảng cách

void setup() {
/*------------------- LCD ----------------------------*/
  SPI.begin();    
  mfrc522.PCD_Init();
  lcd.init();  
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(4,1);
  lcd.print("SLEEPING");
  delay(4);    

  xung = 0;
/*--------------- RIGHT MOTOR ------------------------*/
  pinMode(2, INPUT_PULLUP); //Chan ngat
  pinMode(4, INPUT_PULLUP); //Chan doc encoder
  pinMode(3, OUTPUT); //Chan PWM
  pinMode(10, OUTPUT); //Chan DIR1
  pinMode(11, OUTPUT); //Chan DIR2

  Tocdodat_R = 0; tocdo_R = 0;
  E_R = 0; E1_R = 0; E2_R = 0;
  Output_R = 0; LastOutput_R = 0;
  //Thong so PID
  T = 0.01; //Thoi gian lay mau
  Kp = 1.45; Kd = 0.0; Ki = 0.035;
/*--------------- LEFT MOTOR ------------------------*/
  pinMode(18, INPUT_PULLUP); //Chan ngat
  pinMode(6, INPUT_PULLUP); //Chan doc encoder
  pinMode(5, OUTPUT); //Chan PWM
  pinMode(12, OUTPUT); //Chan DIR1
  pinMode(13, OUTPUT); //Chan DIR2

  Tocdodat_L = 0; tocdo_L = 0;
  E_L = 0; E1_L = 0; E2_L = 0;
  Output_L = 0; LastOutput_L = 0;
/*------------------- INTERRUPT ----------------------*/
  Serial.begin(115200);
  attachInterrupt(0, Demxung_R, FALLING);
  attachInterrupt(5, Demxung_L, FALLING);
  Timer1.initialize(10000);
  Timer1.attachInterrupt(PID);
/*-------------------- MPU6050 ----------------------*/
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
/*-------------------- HC-SR04 ----------------------*/
  pinMode(trig,OUTPUT);
  pinMode(echo, INPUT);
}
/*---------------- INTERRUPT FUNCTION ------------------*/
void Demxung_R(){
  if(digitalRead(4) == LOW){
    xung_R++;
    xung++;
    }
  else
    xung_R--;
}
void Demxung_L(){
  if(digitalRead(6) == LOW)
    xung_L--;
  else
    xung_L++;
}
/*------------------ HC-SR04 FUNCTION ---------------------*/
void hcsr04(){  
  /* Phát xung từ chân trig */
  digitalWrite(trig,0);   // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(trig,1);   // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(trig,0);   // tắt chân trig
  
  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo. 
  duration = pulseIn(echo,HIGH);  
  // Tính khoảng cách đến vật.
  distance = int(duration/2/29.412);
}
/*------------------ MPU6050 FUNCTION ---------------------*/
void mpu6050(){
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  yaw = yaw + norm.ZAxis * timeStep;
  // Output raw

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}
/*------------------PID FUNCTION ---------------------*/
void PID(){
/*======= RIGHT MOTOR =======*/
  tocdo_R = abs((xung_R/(12*19.2))*(1/T)*60);
  xung_R = 0;
  if(Tocdodat_R > 0)
    E_R = Tocdodat_R - tocdo_R;
  else
    E_R = -Tocdodat_R - tocdo_R;
  alpha = 2*T*Kp + Ki*T*T + 2*Kd;
  beta = T*T*Ki - 4*Kd - 2*T*Kp;
  gamma = 2*Kd;
  Output_R = (alpha*E_R + beta*E1_R + gamma*E2_R + 2*T*LastOutput_R)/(2*T);
  LastOutput_R = Output_R;
  E2_R = E1_R;
  E1_R = E_R;
  if(Output_R > 255)
    Output_R=255;
  if(Output_R < 0 || Tocdodat_R == 0)
    Output_R = 0;
    
  if(Tocdodat_R > 0){
    analogWrite(3, Output_R);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
  }
  else{
    analogWrite(3, Output_R);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
  }  
/*========= LEFT MOTOR ==========*/
  tocdo_L = abs((xung_L/(12*19.2))*(1/T)*60);
  xung_L = 0;
  if(Tocdodat_L > 0)
    E_L = Tocdodat_L - tocdo_L;
  else
    E_L = -Tocdodat_L - tocdo_L;
  Output_L = (alpha*E_L + beta*E1_L + gamma*E2_L + 2*T*LastOutput_L)/(2*T);
  LastOutput_L = Output_L;
  E2_L = E1_L;
  E1_L = E_L;
  if(Output_L > 255)
    Output_L=255;
  if(Output_L < 0 || Tocdodat_L == 0)
    Output_L = 0;
    
  if(Tocdodat_L > 0){
    analogWrite(5, Output_L);
    digitalWrite(12, LOW);
    digitalWrite(13, HIGH);
  }
  else
  {
    analogWrite(5, Output_L);
    digitalWrite(12, HIGH);
    digitalWrite(13, LOW);
  }
/*========= CONTROL MOTOR ===========*/ 
}
/*------------------ MOVE STRAIGHT -------------------------*/
void gostraight(){
  /*int dem = 0;
  if(distance > 10){
    if(yaw_t - yaw < -1) Tocdodat_L = Tocdodat_L + 4;
    if(yaw_t - yaw > 1){
      if(Tocdodat_L < 0) Tocdodat_L == 0;
      else 
        Tocdodat_L = Tocdodat_L - 4;
    }
    if(yaw_t - yaw > -1 && yaw_t - yaw < 1) Tocdodat_L = 100;
  }
  while(distance < 10){
    Tocdodat_L = Tocdodat_R = 0;
    dem = 1;
    hcsr04();
  }
  if(dem == 1){
    Tocdodat_L = 100;
    Tocdodat_R = 102;
  }*/
  mpu6050();
   if(yaw_t - yaw < -1) Tocdodat_L = Tocdodat_L - 4;
   if(yaw_t - yaw > 1) Tocdodat_L = Tocdodat_L + 4;
   if(yaw_t - yaw > -1 && yaw_t - yaw < 1) Tocdodat_L = 150;
}

void test(){
  Tocdodat_R = 152;
  Tocdodat_L = 150;
}
/*------------------ MOVE LEFT -------------------------*/
void left(){
  Tocdodat_R = 72;
  Tocdodat_L = -70;
}
/*------------------ MOVE RIGHT -------------------------*/
void right(){
  Tocdodat_R = -72;
  Tocdodat_L = 70;
}
/*------------------ STOP -------------------------*/
void m_stop(){
  Tocdodat_R = 0;
  Tocdodat_L = 0;
}
/*------------------ SPEED UP -------------------------*/
void speed_up(){
  for(int i = 0; i<151; i++){
    Tocdodat_L = i;
    Tocdodat_R = i;
    delay(10);
  }
  Tocdodat_L = 150;
  Tocdodat_R = 152;
}
/*------------------ SPEED DOWN -------------------------*/
void speed_down(){
  for(int i = 150; i>0; i--){
    Tocdodat_L = Tocdodat_L--;
    Tocdodat_R = Tocdodat_R--;
    Serial.println(xung);
    delay(10);
  }
  Tocdodat_L = 0;
  Tocdodat_R = 0;
}
/*------------------ MOVE TO ROOM -------------------------*/
void room(){
  //------------------- Đi thẳng ---------------------------------
  xung=0;
  yaw_t = 0;
  speed_up();
  Tocdodat_L = 150; 
  Tocdodat_R = 152;
  while(xung < 3000){
    mpu6050();
    gostraight();
    Serial.println(yaw);
  }
  m_stop();
  delay(2000);
  //------------------ Quẹo trái = -85 độ -----------------------------
  while(yaw > -82){ 
    left();
    mpu6050();
    Serial.println(yaw);
  }
  xung = 0;
  m_stop();
  delay(2000);
  //------------------ Đi thẳng đến phòng 101 --------------------------
  yaw_t = -82;
  speed_up();
  Tocdodat_L = 150; Tocdodat_R = 152;
  while(xung<8000){
    mpu6050();
    gostraight();
    Serial.println(yaw);
  }
  m_stop();
  delay(2000);
  //------------------- Quẹo phải = 85 độ ----------------------------
  while(yaw < 82){ 
    right();
    mpu6050();
    Serial.println(yaw);
  }
  xung = 0;
  m_stop();
}

void backhome(){
    //------------------- Đi về khúc cua -----------------------------
  yaw_t = 82;
  speed_up();
  Tocdodat_L = 150; Tocdodat_R = 152;
  while(xung<8000){
    mpu6050();
    gostraight();
    Serial.println(yaw);
  }
  m_stop();
  delay(2000);
  //------------------ Quẹo phải = 170 độ -----------------------------
  while(yaw < 161){ 
    right();
    mpu6050();
    Serial.println(yaw);
  }
  xung = 0;
  m_stop();
  delay(2000);
  //------------------- Đi về home -----------------------------
  yaw_t = 161;
  speed_up();
  Tocdodat_L = 150; Tocdodat_R = 152;
  while(xung<3000){
    mpu6050();
    gostraight();
    Serial.println(xung);
  }
  m_stop();
  delay(2000);
  //------------------ Quẹo trai = 0 độ -----------------------------
  while(yaw > -0.5){ 
    left();
    mpu6050();
    Serial.println(yaw);
  }
  xung = 0;
  m_stop();
}
/*------------------ CHOICE ROOM -------------------------*/
void nhapsophong()
{
  int dem=0;
  char sot;
  char sp1;
  char sp2;
  char type;
  while (true)
    {
      char customKey = customKeypad.getKey();
      const char cuskey = customKey;
      lcd.blink();
      delay(200);
      lcd.noBlink();
      if (customKey)
      { 
          if(dem==0)
          {
            sot = cuskey;
            lcd.print(sot);
            lcd.setCursor(5,3);
            dem++;
          }
          else if(dem==1)
          {
            sp1 = cuskey;
            lcd.print(sp1);
            dem++;
          }
          else if(dem==2)
          {
            sp2 =cuskey;
            lcd.print(sp2);
            dem++;
          }
          if(dem==3)
          {
            break;
          }
      }
    }
    sophongstr= sp1;
    sophongstr+=sp2;
    sophongint=sophongstr.toInt();
    sotangstr = sot;
    sotangint=sotangstr.toInt();
}
/*------------------ MAIN FUNCTION -------------------------*/
void loop(){
  // đọc rfid
    String cardid="";
    if ( ! mfrc522.PICC_IsNewCardPresent()) {
      return;
    }
    if ( ! mfrc522.PICC_ReadCardSerial()) {
      return;
    }
    for(byte i=0;i<mfrc522.uid.size;i++)
    {
        cardid.concat(String(mfrc522.uid.uidByte[i]<0x10?" 0":" "));
        cardid.concat(String(mfrc522.uid.uidByte[i],HEX));
    }
    cardid.toUpperCase();
    //xong đọc rfid
    //kiem tra id cua the, neu dung thi moi tiep tuc
    if(cardid.substring(1)=="E1 40 C8 02")    
    {
      //bước nhập số phòng
      lcd.clear();
      lcd.setCursor(1,1);
      lcd.print("ACCESS ACEPTED");
      lcd.setCursor(-4,2);
      lcd.print("SO Tang:");
      lcd.setCursor(-4,3);
      lcd.print("SO Phong:");
      lcd.setCursor(4,2);
      nhapsophong(); // gọi function nhập số phòng

      while(true){
        char customKey = customKeypad.getKey();
        lcd.blink();
        delay(200);
        lcd.noBlink();
        if(customKey == 'A'){
          lcd.clear();
          lcd.setCursor(4,1);
          lcd.print("RUNNING!");
          break;
        }
      }
      // code di chuyển vào đây
      room();
      lcd.clear();
      lcd.setCursor(2,1);
      lcd.print("PLZ, TAKE FOOD <3");
      
      while(true){
        char customKey = customKeypad.getKey();
        if(customKey == 'B'){
        lcd.clear();
        lcd.setCursor(2,1);
        lcd.print("RUNNING BACK");
        break;
        }
      }
      backhome();
      lcd.clear();
      lcd.setCursor(4,1);
      lcd.print("SLEEPING");
    }
    else // nếu không đúng cardid thì không thực hiện, chỉ in ra từ chối rồi về sleeping
    {
      lcd.clear();
      lcd.setCursor(-3,2);
      lcd.print("ACCESS  DENIED");
      delay(1000);
      lcd.clear();
      lcd.setCursor(4,1);
      lcd.print("SLEEPING");
    }
}
