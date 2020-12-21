#include <Servo.h>
Servo servo;

const int SERVO_PIN = 9;      // 서보모터1 연결핀
const int IR_R = A4;  //  적외선센서 우측 핀
const int IR_L = 4;  // 적외선센서 좌측 핀

const int M1_PWM = 5;   // DC모터1 PWM 핀 왼
const int M1_DIR1 = 7;   // DC모터1 DIR1 핀
const int M1_DIR2 = 8;   // DC모터 1 DIR2 핀

const int M2_PWM = 6;   // DC모터2 PWM 핀
const int M2_DIR1 = 11;   // DC모터2 DIR1 핀
const int M2_DIR2 = 12;   // DC모터2 DIR2 핀

const int FC_TRIG  = 13;   // 전방 초음파 센서 TRIG 핀
const int FC_ECHO = 10;  // 전방 초음파 센서 ECHO 핀
const int L_TRIG = A2;  // 좌측 초음파 센서 TRIG 핀
const int L_ECHO = A0;  // 좌측 초음파 센서 ECHO 핀
const int R_TRIG = A5;   // 우측 초음파 센서 TRIG 핀
const int R_ECHO = A1;  // 우측 초음파 센서 ECHO 핀

const int MAX_DISTANCE = 2000; // 초음파 센서의 최대 감지거리

//************************************start
int step=0;
boolean tfstart=false;
//************************************end

float center;
float left;
float right;

int state = 0;
// 자동차 튜닝 파라미터 =====================================================================
boolean detect_ir = true; // 검출선이 흰색 = false, 검정색 = true

//여긴 안바꿈
int punch_pwm = 200; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 50; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec) 


//여길 적당히 바꿔보기
int max_ai_pwm = 140; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 70; // 자율주행 모터 최소 출력 (0 ~ 255)

//angle_offset=0;
//limit 각도 제한  (65-70정도까지 가면 안움직이는 거 같음 )
int angle_offset = -15; // 서보 모터 중앙각 오프셋 (단위: 도) 마이너스 값에서 바퀴가 왼쪽으로 정렬되고 플러스 값에서 오른쪽으로 정렬됨
int angle_limit = 55; // 서보 모터 회전 제한 각 (단위: 도)


float cur_steering;
float cur_speed;
float compute_steering;
float compute_speed;

float max_pwm;
float min_pwm;

// 초음파 거리측정
float GetDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH, 5000);
  if (duration == 0)
    return MAX_DISTANCE;
  else
    return duration * 0.17;     // 음속 340m/s
}

// 앞바퀴 조향
void SetSteering(float steering)
{
  cur_steering = constrain(steering, -1, 1);// constrain -1~ 1 값으로 제한

  float angle = cur_steering * angle_limit;
  int servoAngle = angle + 90;
  servoAngle += angle_offset;

  servoAngle = constrain(servoAngle, 0, 180);
  servo.write(servoAngle);
}


// 뒷바퀴 모터회전
void SetSpeed(float speed)
{
  speed = constrain(speed, -1, 1);

  if ((cur_speed * speed < 0) // 움직이는 중 반대 방향 명령이거나
      || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
  {
    cur_speed = 0;
    digitalWrite(M1_PWM, HIGH);
    digitalWrite(M1_DIR1, LOW);
    digitalWrite(M1_DIR2, LOW);

    digitalWrite(M2_PWM, HIGH);
    digitalWrite(M2_DIR1, LOW);
    digitalWrite(M2_DIR2, LOW);

    if (stop_time > 0)
      delay(stop_time);
  }

  if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
  {
    if (punch_time > 0)
    {
      if (speed > 0)
      {
        analogWrite(M1_PWM, punch_pwm);
        digitalWrite(M1_DIR1, HIGH);
        digitalWrite(M1_DIR2, LOW);

        analogWrite(M2_PWM, punch_pwm);
        digitalWrite(M2_DIR1, HIGH);
        digitalWrite(M2_DIR2, LOW);
      }
      else if (speed < 0)
      {
        analogWrite(M1_PWM, punch_pwm);
        digitalWrite(M1_DIR1, LOW);
        digitalWrite(M1_DIR2, HIGH);

        analogWrite(M2_PWM, punch_pwm);
        digitalWrite(M2_DIR1, LOW);
        digitalWrite(M2_DIR2, HIGH);
      }
      delay(punch_time);
    }
  }

  if (speed != 0) // 명령이 정지가 아니라면
  {
    int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm;           // 0 ~ 255로 변환

    if (speed  > 0)
    {
      analogWrite(M1_PWM, pwm);
      digitalWrite(M1_DIR1, HIGH);
      digitalWrite(M1_DIR2, LOW);

      analogWrite(M2_PWM, pwm);
      digitalWrite(M2_DIR1, HIGH);
      digitalWrite(M2_DIR2, LOW);
    }
    else if (speed < 0)
    {
      analogWrite(M1_PWM, pwm);
      digitalWrite(M1_DIR1, LOW);
      digitalWrite(M1_DIR2, HIGH);

      analogWrite(M2_PWM, pwm);
      digitalWrite(M2_DIR1, LOW);
      digitalWrite(M2_DIR2, HIGH);
    }
  }
  cur_speed = speed;
}

//==================평행주차===========================================================
void Parallel(){ 
  boolean a = false;
  boolean b = false;
  boolean c = false;
  boolean d = false;
  boolean e = false;
  boolean finish = false;
  
  while (finish==false){
  
    float center = GetDistance(FC_TRIG, FC_ECHO);
    float left = GetDistance(L_TRIG, L_ECHO);
    float right = GetDistance(R_TRIG, R_ECHO);

    Serial.print("C:");
    Serial.print(center);
  
    Serial.print("     L:");
    Serial.print(left);
    
    Serial.print("     R:");
    Serial.println(right);

    if(a==false && b==false && c==false && d==false && e==false)
    {
      //15센치 이하 물체감지되면 멈추기
      if (center <= 150) {
        SetSteering(0);
        SetSpeed(0);
      }
    
    //ir 아무것도 감지되지 않을 때 직진 
      else if (digitalRead(IR_R) == detect_ir && digitalRead(IR_L) == detect_ir) { 
        SetSteering(0);
        SetSpeed(0.2);
      }
      //왼쪽 
      else if (digitalRead(IR_R) != detect_ir) { // 오른 쪽 차선이 검출된 경우
        SetSteering(-1);
        SetSpeed(0.1);
      }
    
      else if (digitalRead(IR_L) != detect_ir) { //왼쪽 차선이 검출된 경우
        SetSteering(1);
        SetSpeed(0.1); //내가 정해놓은 속도값의 영점일
      }
    }
      
    if(left<=200 && a==false && right<=200){ //왼쪽값이 30cm 보다 작아지면 a단계 시작
      Serial.println("parallel 시작!");
      a = true;
    }
    
    if(a==true && b==false && c==false && d==false && e==false){
      if (right>150){ // 평행주차할 공간 만나면
        Serial.println("평행주차 공간 감지");
        b=true;
      }
      if (left>=93 && left<=107) { 
        SetSteering(0);
        SetSpeed(0.2);
      }
      else if (left>107) { //오른쪽으로 너무 갔으면
        SetSteering(-0.5);
        SetSpeed(0.1);
      }
      //오른쪽
      else if (left<93) { //왼쪽으로 너무 갔으면
         SetSteering(0.5);
         SetSpeed(0.1); 
      }
    }
    
    if(a==true && b==true && c==false && d==false && e==false){
      if (right<140){ // 평행주차할 공간 지나면
        Serial.println("평행주차 공간 패스 완료");
        c=true;
      }
      if (left>=93 && left<=107) { 
        SetSteering(0);
        SetSpeed(0.2);
      }
      else if (left>107) { //오른쪽으로 너무 갔으면
        SetSteering(-0.5);
        SetSpeed(0.1);
      }
      //오른쪽
      else if (left<93) { //왼쪽으로 너무 갔으면
         SetSteering(0.5);
         SetSpeed(0.1); 
      }
    }
    
    if(a==true && b==true && c==true && d==false && e==false){
      unsigned long time=millis();
      Serial.println("삼초동안 직진");
      while(millis()-time<700){ //0.7초동안
        center = GetDistance(FC_TRIG, FC_ECHO);
        left = GetDistance(L_TRIG, L_ECHO);
        right = GetDistance(R_TRIG, R_ECHO);
        
        if (left>=93 && left<=107) { 
          SetSteering(0);
          SetSpeed(0.2);
        }
        else if (left>107) { //오른쪽으로 너무 갔으면
          SetSteering(-0.5);
          SetSpeed(0.1);
        }
        //오른쪽
        else if (left<93) { //왼쪽으로 너무 갔으면
           SetSteering(0.5);
           SetSpeed(0.1); 
        }
      }
      
      unsigned long time2=millis();
      
      while(millis()-time2<900){ //0.7초동안
        center = GetDistance(FC_TRIG, FC_ECHO);
        left = GetDistance(L_TRIG, L_ECHO);
        right = GetDistance(R_TRIG, R_ECHO);
        
        if (left>=93 && left<=120) { 
          SetSteering(0);
          SetSpeed(-0.2);
        }
        else if (left>120) { //오른쪽으로 너무 갔으면
          SetSteering(-0.5);
          SetSpeed(-0.1);
        }
        //오른쪽
        else if (left<93) { //왼쪽으로 너무 갔으면
           SetSteering(0.5);
           SetSpeed(-0.1); 
        }
      }
      d=true;
      Serial.println("직진까지 완료");
    }

    
    if(a==true && b==true && c==true && d==true && e==false){ //하드코드 시작\
      
      //핸들 오른쪽으로
      SetSteering(1);
      SetSpeed(-0.3); // 0 ~ 255의 PWM값으로 속도 조절
      delay(1100); //오른쪽 후진 몇초
        
      SetSteering(-1);
      SetSpeed(-0.3);   // 0 ~ 255의 PWM값으로 속도 조절
      delay(800); //왼쪽후진몇초
      
      SetSteering(-0.2);
      SetSpeed(-0.2);   // 0 ~ 255의 PWM값으로 속도 조절
      delay(145); //왼쪽후진몇초
      
      SetSteering(0);
      SetSpeed(-0.2);   // 0 ~ 255의 PWM값으로 속도 조절
      delay(90); 
      
      SetSteering(0);
      SetSpeed(0);
      delay(1000); //정지 2초
    
      SetSteering(-1);
      SetSpeed(0.3);   // 0 ~ 255의 PWM값으로 속도 조절
      delay(50); //왼쪽후진몇초

      SetSteering(-1);
      SetSpeed(0.3);   // 0 ~ 255의 PWM값으로 속도 조절
      delay(830); //왼쪽후진몇초
    
      SetSteering(1);
      SetSpeed(0.3);   // 0 ~ 255의 PWM값으로 속도 조절
      delay(1000); //오른쪽후진몇초

      e=true;
      
    }
    
    if(a==true && b==true && c==true && d==true && e==true){
       unsigned long time3=millis();
      Serial.println("삼초동안 직진");
      while(millis()-time3<700){ //0.7초동안
        center = GetDistance(FC_TRIG, FC_ECHO);
        left = GetDistance(L_TRIG, L_ECHO);
        right = GetDistance(R_TRIG, R_ECHO);
        
        if (left>=80 && left<=120) { 
          SetSteering(0);
          SetSpeed(0.2);
        }
        else if (left>120) { //오른쪽으로 너무 갔으면
          SetSteering(-1);
          SetSpeed(0.1);
        }
        //오른쪽
        else if (left<80) { //왼쪽으로 너무 갔으면
           SetSteering(1);
           SetSpeed(0.1); 
        }
      }
      
      finish=true;
    }
  }
}

//======================교차로==============================================
void Intersection(){
  while(true){
    if (digitalRead(IR_R) != detect_ir && digitalRead(IR_L) != detect_ir){
      SetSpeed(0);
      delay(1000);
      break;
    }
    compute_steering = cur_steering;
    compute_speed = cur_speed;
    
  
  //15센치 이하 물체감지되면 멈추기
    if (center <= 150) {
      compute_steering = 0;
      compute_speed = 0;
    }
  
  //ir 아무것도 감지되지 않을 때 직진 
    else if (digitalRead(IR_R) == detect_ir && digitalRead(IR_L) == detect_ir) { 
      compute_steering = 0;
      compute_speed = 0.5;
    }
    //왼쪽 
    else if (digitalRead(IR_R) != detect_ir) { // 오른 쪽 차선이 검출된 경우
      compute_steering = -1;
      compute_speed = 0.1;
    }
  
    else if (digitalRead(IR_L) != detect_ir) { //왼쪽 차선이 검출된 경우
      compute_steering = 1;
      compute_speed = 0.1; //내가 정해놓은 속도값의 영점일
    }
  
    SetSpeed(compute_speed);
    SetSteering(compute_steering);
    
  }
}

//============================후진주차====================
void backParking(){  
  SetSpeed(0.05);
  SetSteering(-1);
  SetSteering(0);

  boolean first_step = true; 
  boolean second_step = true;
  boolean second_2step= true;
  boolean third_step = true;
  
  while(true){
  
    if(GetDistance(FC_TRIG, FC_ECHO)<120&& first_step == true){ //만약 앞까지 오면, 뒤까지 간격 벌어질 때까지 오른쪽으로 틀어서 후진하기. 
      boolean break_out = true;
      
      while(break_out){
        Serial.print("first");
        SetSteering(1);
        SetSpeed(-0.3);
        if(GetDistance(FC_TRIG, FC_ECHO)>180){
          first_step = false;
          break_out = false;
        }
      }
    }
   
    if(first_step == false && second_step ==true){ //
      while(true){
        Serial.print("second");
         SetSteering(0);
         SetSpeed(0.05);
         if(GetDistance(FC_TRIG, FC_ECHO)<210){
          second_step = false;
          break;
         }
      }
    }
    if(second_step == false && second_2step ==true){
      while(true){
        Serial.print("second_2222");
        Serial.print(GetDistance(FC_TRIG, FC_ECHO));
        SetSteering(-1);
        SetSpeed(0.1);
        delay(1700);
        if(GetDistance(FC_TRIG, FC_ECHO)>1200  ){
          Serial.print("stop7777777777777777777777777777777777777777777777777777777777777777777777777777777777777");
          SetSteering(0);
          SetSpeed(0);
          second_2step = false;
         break;
        }
      }
  }

  if(second_2step == false && third_step ==true){
    Serial.print("third");
    SetSteering(0.2);
    SetSpeed(0.1);
    delay(100); 
    
    boolean backPark = false;
    while(true){
      float center = GetDistance(FC_TRIG, FC_ECHO);
      float left = GetDistance(L_TRIG, L_ECHO);
      float right = GetDistance(R_TRIG, R_ECHO);
      if (right>=93 && right<=120) { 
        SetSteering(0);
        SetSpeed(-0.2);
      }
      else if (right>120) { //오른쪽 멀어지면
        SetSteering(0.5);
        SetSpeed(-0.1);
      }
      //오른쪽
      else if (right<93) { //오른쪽가까워지면
         SetSteering(-0.5);
         SetSpeed(-0.1); 
      }
        
        
        if(left >1800){
          backPark= true;
         
        }
        if(backPark ==true && GetDistance(L_TRIG, L_ECHO)<200){ 
          delay(700);
          SetSpeed(0);
          delay(2000);
          SetSteering(0);
          SetSpeed(0.8);
          delay(500);
          return;
          }
        }
      }
   }
}

//===========================장피========================

void ObstacleAvoid(){
  center = GetDistance(FC_TRIG, FC_ECHO);
  left = GetDistance(L_TRIG, L_ECHO);
  right = GetDistance(R_TRIG, R_ECHO);
    

  //정지
  SetSpeed(0);
  delay(50);
  
  //후진
  SetSpeed(-0.2);
  delay(500);
  
  //정지
  SetSpeed(0);
  delay(100);
  
  //왼쪽틀기
  SetSteering(-1);
  SetSpeed(0.3);
  delay(300);     
  SetSteering(1);  
  delay(100);
  SetSteering(0);   
  //직진
  SetSpeed(0.1);

    
}

void Avoid(){
  boolean Right = true; 
  unsigned long time4=millis();
  boolean avoid=false;
  boolean turn_one=false;
  
  while(true){
  
    Serial.print("start\n");
    
    if(GetDistance(FC_TRIG, FC_ECHO)<150){
       ObstacleAvoid();
       ObstacleAvoid();
       time4=millis();
       avoid=true;
       }
    
    else if(digitalRead(IR_R) == detect_ir && digitalRead(IR_L) == detect_ir){
       SetSteering(0);
       SetSpeed(0.1);
    }
    
    else if (digitalRead(IR_R) != detect_ir) { // 오른쪽 차선이 검출된 경우
        SetSteering(-1);
        SetSpeed(0.1);
      }
    else if (digitalRead(IR_L) != detect_ir) { //왼쪽 차선이 검출된 경우
       SetSteering(1);
       SetSpeed(0.5);
    }
    if(avoid==true && turn_one==false){
      if (digitalRead(IR_L) != detect_ir) { //왼쪽 차선이 검출된 경우
        SetSteering(1);
        SetSpeed(0.5);
        delay(1500);
        turn_one=true;
      }
    }
    
    if(millis()-time4>3200){
      return;
    }
  }

}

void lastpang(){
  while(true){
        //15센치 이하 물체감지되면 멈추기
        if (center <= 150) {
          SetSteering(0);
          SetSpeed(0);
        }
      
      //ir 아무것도 감지되지 않을 때 직진 
        else if (digitalRead(IR_R) == detect_ir && digitalRead(IR_L) == detect_ir) { 
          SetSteering(0);
          SetSpeed(0.2);
        }
        //왼쪽 
        else if (digitalRead(IR_R) != detect_ir) { // 오른 쪽 차선이 검출된 경우
          SetSteering(-1);
          SetSpeed(0.1);
        }
      
        else if (digitalRead(IR_L) != detect_ir) { //왼쪽 차선이 검출된 경우
          SetSteering(1);
          SetSpeed(0.5);
          delay(700);//내가 정해놓은 속도값의 영점일
          return;
        }
  }
}

void setup() {
    
  Serial.begin(115200);
  servo.attach(SERVO_PIN); //서보모터 초기화

  pinMode(IR_R, INPUT);
  pinMode(IR_L, INPUT);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);

  pinMode(FC_TRIG, OUTPUT);
  pinMode(FC_ECHO, INPUT);
  pinMode(L_TRIG, OUTPUT);
  pinMode(L_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);
  
  max_pwm = max_ai_pwm;
  min_pwm = min_ai_pwm;

  SetSteering(0);
  SetSpeed(0);
}

void loop() {

  compute_steering = cur_steering;
  compute_speed = cur_speed;

  center = GetDistance(FC_TRIG, FC_ECHO);
  left = GetDistance(L_TRIG, L_ECHO);
  right = GetDistance(R_TRIG, R_ECHO);

  Serial.print("C:");
  Serial.print(center);

  Serial.print("     L:");
  Serial.print(left);
  
  Serial.print("     R:");
  Serial.println(right);
  

//15센치 이하 물체감지되면 멈추기
  if (center <= 150) {
    compute_steering = 0;
    compute_speed = 0;
  }

//ir 아무것도 감지되지 않을 때 직진 
  else if (digitalRead(IR_R) == detect_ir && digitalRead(IR_L) == detect_ir) { 
    compute_steering = 0;
    compute_speed = 0.3;
  }
  //왼쪽 
  else if (digitalRead(IR_R) != detect_ir) { // 오른 쪽 차선이 검출된 경우
    compute_steering = -1;
    compute_speed = 0.1;
  }

  else if (digitalRead(IR_L) != detect_ir) { //왼쪽 차선이 검출된 경우
    compute_steering = 1;
    compute_speed = 0.1; //내가 정해놓은 속도값의 영점일
  }

  SetSpeed(compute_speed);
  SetSteering(compute_steering);

  if (digitalRead(IR_R) != detect_ir && digitalRead(IR_L) != detect_ir){
    if(step==0){
      Parallel();
      step+=1;
    }
    else if(step==1){
      //*************교차로***************
      SetSpeed(0);
      delay(2000);
      SetSteering(0);
      SetSpeed(0.5);
      delay(200);
      Intersection();
      step+=1;
      SetSpeed(0.2);
    }
    else if(step==2){
      backParking();
      step+=1;
    }
    else if(step==3){
      Avoid();
      step+=1;
    }
    //종료시 벽에 충돌하지 않고 5cm이내 정차 (범퍼와 벽사이)
  }

  delay(100);

}
