#include "genral_exec.h"
//#define DEBUG_PLUS
//#define DEBUG_GPS
//#define NOGPSDEBUG
//#define DEBUG_WATER
//#define GPSTRACKER
#define NOGPS_DATE 0,9,9,16,07,2022
#ifdef NOGPSDEBUG
unsigned nogpsdate[][6] = {{23,57,9,15,07,2022},{23,57,10,15,07,2022},{23,59,9,15,07,2022},{0,1,9,16,07,2022},{0,3,9,16,07,2022},{0,5,9,16,07,2022},{0,7,9,16,07,2022},{0,9,9,16,07,2022}};
int nogps_i=0;
#endif
#ifdef NOGPSDEBUG
#define TIME_UPDATE 3600
#else
#define TIME_UPDATE 3600
#endif 
#define water_btn 21
#define orange_btn 13
#define yellow_btn 39
#define green_btn 34
#define blue_btn 35
#define white_btn 32
#define violet_btn 33

#define lightgreen_rel 5
#define brown_rel 15
#define orange_rel 18
#define yellow_rel 4
#define green_rel 19
#define blue_rel 14
#define white_rel 27
#define violet_rel 26
int relays[]={violet_rel,white_rel,blue_rel,green_rel,yellow_rel,orange_rel,brown_rel,lightgreen_rel};
int btns[]={violet_btn,white_btn,blue_btn,green_btn,yellow_btn,orange_btn/*,water_btn*/};

#include <DHT.h>      // подключаем библиотеку для датчика
DHT dht(22, DHT11);

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#include <Time.h>
//#include <TimeLib.h>
#include <TimeAlarms.h>

//#include <TinyGPS.h>
#include "TinyGPS++.h"
TinyGPSPlus gps;
//TinyGPS gps; 
const int offset = 3;
#define GPS_PIN_TX 17
#define GPS_PIN_RX 16
time_t prevDisplay = 0;

AlarmId id;

#include <CircularBuffer.h>

CircularBuffer<char, 1200> buffer;

volatile unsigned long pulseCount;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}
void dprint(String s,byte log=1)
{ char bb[50];
  Serial.print(s);
  SerialBT.print(s);
//  sprintf(bb,"%s",s);
 if(log==1)
  {
  s.toCharArray(bb, 50); 
  for(int i=0;i<s.length();i++)
   {
     buffer.push(bb[i]);
   }
  }
}
void dprintln(String s,byte log=1)
{
  char bb[50];
  Serial.println(s);
  SerialBT.println(s);
//   sprintf(bb,"%s\n",s); 
 if(log==1)
  {
  s.toCharArray(bb, 50); 
  for(int i=0;i<s.length();i++)
   {
    buffer.push(bb[i]);
   }
    buffer.push('\n');
  }
}
void dprint(long s,byte log=1)
{
  dprint(String(s),log);
  /*char bb[50];
  Serial.print(s);
  SerialBT.print(s);*/
/*  sprintf(bb,"%s",s); 
  for(int i=0;i<String(s).length()+1;i++)
   {
    buffer.push(bb[i]);
   }*/
}
void dprintln(long s,byte log=1)
{
  dprintln(String(s),log);
 /* char bb[50];
  Serial.println(s);
  SerialBT.println(s);
  sprintf(bb,"%d\n",s); */
 /*   for(int i=0;i<String(s).length()+1;i++)
   {
    buffer.push(bb[i]);
   }*/
}

void printDigits(int digits,byte log=1)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    dprint("0",log);
  dprint(digits,log);
}



void digitalClockDisplay(){
  // digital clock display of the time
//  dprint("time ajust to: ");
  dprint(hour());
  dprint(":");
  printDigits(minute());
  dprint(":");
  printDigits(second());
  dprint(" ");
  dprint(day());
  dprint(".");
  dprint(month());
  dprint(".");
  dprint(year()); 
  dprint(" ");
  dprint(weekday()); 
  dprintln(""); 
}

void digitalClockDisplay(time_t t,byte log=1){

  dprint(hour(t),log);  dprint(":",log);
  printDigits(minute(t),log);  dprint(":",log);
  printDigits(second(t),log);
  dprint(" ",log);
  printDigits(day(t),log);
  dprint(".",log);
  printDigits(month(t),log);
  dprint(".",log);
  dprint(year(t),log); 
  dprint(" ",log);
  dprint(weekday(t),log); 
  dprintln("",log); 
}
 


class general_init: public general_do
{
  public:

 
   virtual void handle(task *x) {
 
    digitalClockDisplay();
    dprint (x->taskname);dprintln(" init");
    pinMode(x->relay, OUTPUT);
    pinMode(x->btn, INPUT);
    digitalWrite(x->relay, HIGH);
    x->stat=0;
    //todo добавление в расписание
    if(x->StDayofWeek>0)
     {
     x->aID=Alarm.alarmRepeat((timeDayOfWeek_t)x->StDayofWeek,x->StHour,x->StMin,0,doalarm);
     }
    else
     {
     if(x->StHour>0)
      {
       x->aID=Alarm.alarmRepeat(x->StHour,x->StMin,0,doalarm);   
      }
     }
 //   Alarm.alarmRepeat(x->StHour,x->StMin,0,doalarm);
   }
} generalInit;

class general_start: public general_do
{
  public:
   virtual void handle(task *x) {
    digitalClockDisplay();
    dprint(x->taskname);dprintln(" started");
    x->finish=x->duration+millis();
    digitalWrite(x->relay,LOW);
    digitalWrite(lightgreen_rel,LOW);
    digitalWrite(brown_rel,LOW);
    x->stat=1;
    pulseCount=0;
   }
} generalStart;

class general_exec: public general_do
{
  public:
   virtual void handle(task *x) {
#ifdef DEBUG_MINUS   
     dprint(x->taskname);dprintln(" exec");
#endif
    digitalWrite(x->relay,LOW);
    digitalWrite(lightgreen_rel,LOW);
    digitalWrite(brown_rel,LOW);
   }
} generalExec;

class general_fin: public general_do
{
  public:
   virtual void handle(task *x) {
    if(x->stat!=0)
     {
    digitalClockDisplay();
    dprint(x->taskname);dprintln(" stoped");
    dprint("water:");dprintln(String(pulseCount*2.25/1000.));
    digitalWrite(x->relay,HIGH);
    digitalWrite(lightgreen_rel,HIGH);
    digitalWrite(brown_rel,HIGH);
    x->stat=0;
     }
   }
} generalFin;

//------------------------------------------------------------------------------
class stop_init: public general_do
{
  public:
   virtual void handle(task *x) {
 
    digitalClockDisplay();
    dprint (x->taskname);dprintln(" init");
    pinMode(x->btn, INPUT);
   }
} stopInit;

class stop_start: public general_do
{
  public:
   virtual void handle(task *x) {
    digitalClockDisplay();
    dprint(x->taskname);dprintln(" emergancy STOP");
    emergency_stop();
   }
} stopStart;

class stop_exec: public general_do
{
  public:
   virtual void handle(task *x) {

   }
} stopExec;

class stop_fin: public general_do
{
  public:
   virtual void handle(task *x) {

   }
} stopFin;




//----------------------------------------------------------------------------






    // PushButton::PushButton(int8_t _pin, Event * _event,String _msg) : pin(_pin), state(true), event(_event),msg(_msg)

task::task(String _taskname,int _btn,int _relay,int8_t _StHour,int8_t _StMin,int8_t _StDayofWeek,unsigned long _duration,unsigned long _finish,general_do * _init,general_do * _start,general_do * _exec,general_do * _fin)
   :taskname(_taskname),btn(_btn),relay(_relay),StHour(_StHour),StMin(_StMin),StDayofWeek(_StDayofWeek),duration(_duration),finish(_finish),init(_init),start(_start),exec(_exec),fin(_fin)
{
#ifdef DEBUG_PLUS
dprintln("--------------------init--------------"); 
#endif
aID=-1;
}

void task::check()
{
  if(digitalRead(btn)==HIGH)
   {
    start->handle(this);
   }
  else
   {
    if(finish>millis())
     {
#ifdef DEBUG_MINUS
dprintln(finish-millis());
#endif
      exec->handle(this);
 
     }
    else
     {
      fin->handle(this);
     }
   }
}
//int relays[]={violet_rel,white_rel,blue_rel,green_rel,yellow_rel,orange_rel,brown_rel,lightgreen_rel};
#define v1_3bochki 660000
#define v1_2bochki 1000000
#define v2_3bochki 1320000

#ifdef GPSTRACKER
task tasks_arr[]={
  {"Violet task",violet_btn,violet_rel,10,0,6,v1_2bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"White task",white_btn,white_rel,10,0,7,v1_2bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"blue task",blue_btn,blue_rel,17,3,-1,30000,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"green task",green_btn,green_rel,10,0,1,v1_2bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"yellow task",yellow_btn,yellow_rel,19,05,-1,v1_3bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"Orange task",orange_btn,orange_rel,-1,-1,-1,0,0,&stopInit,&stopStart,&stopExec,&stopFin}/*,
  {"Water task",water_btn,orange_rel,-1,-1,-1,0,0,&stopInit,&stopStart,&stopExec,&stopFin}*/
  };
#else
task tasks_arr[]={
  {"Violet task",violet_btn,violet_rel,10,0,6,v1_2bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"White task",white_btn,white_rel,10,0,7,v1_2bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"blue task",blue_btn,blue_rel,18,30,-1,v1_3bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"green task",green_btn,green_rel,10,0,1,v1_2bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"yellow task",yellow_btn,yellow_rel,19,05,-1,v1_3bochki,0,&generalInit,&generalStart,&generalExec,&generalFin},
  {"Orange task",orange_btn,orange_rel,-1,-1,-1,0,0,&stopInit,&stopStart,&stopExec,&stopFin}/*,
  {"Water task",water_btn,orange_rel,-1,-1,-1,0,0,&stopInit,&stopStart,&stopExec,&stopFin}*/
  };
#endif
#define USE_SPECIALIST_METHODS
void emergency_stop()
{
    for (task & ntask : tasks_arr) 
     {
     ntask.finish=0;
     ntask.fin->handle(&ntask);
     }

    for (task & ntask : tasks_arr) 
     {
     if(ntask.aID>=0)
      {
     dprint(ntask.aID);dprint(" "); dprint(ntask.taskname);dprint(" ");dprint(Alarm.read(ntask.aID));dprint(" ");
     digitalClockDisplay(Alarm.read(ntask.aID),1);
      }
     }
  setTimefromGPS(60000);
}

void doalarm()
    {
      
      dprintln("Alarm");
      int id;
      id=Alarm.getTriggeredAlarmId();
       for (task & ntask : tasks_arr) 
        {
          if(ntask.aID==id)
           {
            ntask.start->handle(&ntask);
           }
           
        }
    }
byte pushbuffer=0;

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected!");
    pushbuffer=1;
  }
}

void setup() {
Serial.begin(115200);
SerialBT.begin("poliv");
dht.begin();
SerialBT.register_callback(btCallback);
int i;
for (i=0;i<(sizeof(relays)+1)/sizeof(int);i++)
 {
  pinMode(relays[i], OUTPUT);
  digitalWrite(relays[i], HIGH);
 // dprintln(i);
 }
for (i=0;i<(sizeof(btns)+1)/sizeof(int);i++)
 {
  pinMode(btns[i], INPUT);
 }

    Serial1.begin(9600, SERIAL_8N1, GPS_PIN_TX, GPS_PIN_RX);
   Serial1.setRxBufferSize(1024);
   Alarm.delay(500);  
   dprintln("Search GPS..");
   Alarm.delay(1000);
#ifdef GPSTRACKER
 pinMode(23, OUTPUT);
 digitalWrite(GPIO_NUM_23, HIGH);
#endif
setTime(setTimefromGPS(60*30*1000));
setSyncProvider(CsetTimefromGPS);
setSyncInterval(TIME_UPDATE);
for (task & ntask : tasks_arr) ntask.init->handle(&ntask);
//Alarm.timerRepeat(TIME_UPDATE, CsetTimefromGPS);
pinMode(water_btn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(water_btn), pulseCounter, FALLING);

#ifdef DEBUG_MINUS
 emergency_stop();
#endif
}


time_t CsetTimefromGPS()
{
  return(setTimefromGPS(20000));
}


 time_t setTimefromGPS(unsigned long dt)
{
unsigned long st=millis();
 tmElements_t tm;
 time_t tt;
 float hp = dht.readHumidity();
 float tp = dht.readTemperature();
 dprint("Humidity:");  dprint(String(hp));
 dprint("Temperature:");  dprintln(String(tp));
/*if(gps.date.year()>=2022)
 {
 dprint("Now is: ");
 digitalClockDisplay();
 }*/
#ifdef NOGPSDEBUG
//setTime(NOGPS_DATE);
 tm.Year=nogpsdate[nogps_i][5]-1970;
 tm.Month=nogpsdate[nogps_i][4];
 tm.Day=nogpsdate[nogps_i][3];
 tm.Hour=nogpsdate[nogps_i][0];
 tm.Minute=nogpsdate[nogps_i][1];
 tm.Second=nogpsdate[nogps_i][2];
 

  tt=makeTime(tm)+(offset * SECS_PER_HOUR);
  nogps_i++;
 //setTime(tt+offset * SECS_PER_HOUR);
// Alarm.delay(1000);
 dprint("Set DEBUG Time ");

#else
   do
  {
     while (Serial1.available()) {
      char g;
      g = Serial1.read();
#ifdef    DEBUG_GPS
    Serial.print(g);
#endif
      gps.encode(g);
      }
   if((st+dt)<millis())
      return(0);
  }
 while (((!gps.location.isValid()) || (gps.date.year()<2022) || (gps.location.age()>1000)));
 //unsigned long age;
 //int Year;
 //byte Month, Day, Hour, Minute, Second;

/* Year=gps.date.year();
 Month=gps.date.month();
 Day=gps.date.day();
 Hour=gps.time.hour();
 Minute=gps.time.minute();
 Second=gps.time.second();*/
 tm.Year=gps.date.year()-1970;
 tm.Month=gps.date.month();
 tm.Day=gps.date.day();
 tm.Hour=gps.time.hour();
 tm.Minute=gps.time.minute();
 tm.Second=gps.time.second();
 //age=gps.time.age();
 tt=makeTime(tm)+offset * SECS_PER_HOUR;;

 //setTime(t);
 // Alarm.delay(1000);
 dprint("Got time from GPS ");
 #endif
  digitalClockDisplay(tt);
return(tt);

}

 
void loop() {
  int i;
  Alarm.delay(10);
 for (task & ntask : tasks_arr) ntask.check();
 if (Serial1.available()) {
   char g;
   g = Serial1.read();
#ifdef    DEBUG_GPS
    Serial.print(g);
#endif
   gps.encode(g);
   }
 if(pushbuffer==1)
  {
  pushbuffer=0;
  using index_t = decltype(buffer)::index_t;
  for (index_t i = 0; i < buffer.size()-2; i++) {
     SerialBT.print((char)buffer[i]);
     }

      
   for (int j = 0; j < 6; j++) {
    int k=0;
    for (task & ntask : tasks_arr) 
     {
      if(ntask.aID==j)
       {
        dprint(ntask.aID);dprint(" "); dprint(ntask.taskname);dprint(" ");digitalClockDisplay(Alarm.getNextTrigger(j),0);dprint(" ");
        k=1;
        break;
       }
     }
    if(k==0)
      {
       dprint(j);dprint(" ");digitalClockDisplay(Alarm.getNextTrigger(j),0);
      }
      k=0;
     }
   float hp = dht.readHumidity();
   float tp = dht.readTemperature();
   dprint("Humidity:");  dprint(String(hp));
   dprint("Temperature:");  dprintln(String(tp));
  }
 #ifdef DEBUG_WATER
 Serial.println(pulseCount);
 #endif
}
