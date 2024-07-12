#include <vfs_api.h>
#include <FSImpl.h>
#include <FS.h>
#define CONFIGFILE "/main.ini"
#include "genral_exec.h"
#define DEBUG_PLUS
//#define DEBUG_LOOP
//#define DEBUG_MINUS
//#define DEBUG_GPS
//#define NOGPSDEBUG
//#define DEBUG_WATER
//#define DEBUG_VIOLET_BTN
//#define GPSTRACKER
#define BTNTRIM 1000
#define PERMANENT_DURATION 4294967290
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
#define water_btn 25
#define tempr_btn 22

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
#define red_rel 21
#define cyan_rel 23
#define NOFRELAYS sizeof(relays)/sizeof(relays[0])
#define NOFPROCS 10

unsigned long mill_restart=400000000;   //интервал автоматической перезагрузки
int relays[]={violet_rel,white_rel,blue_rel,green_rel,yellow_rel,red_rel,orange_rel,brown_rel,lightgreen_rel,cyan_rel};
#define NOFBTNS sizeof(btns)/sizeof(btns[0])
int btns[]={violet_btn,white_btn,blue_btn,green_btn,yellow_btn,orange_btn/*,water_btn*/};
String btcmd="";
float sensors[3];  // Значения татчиков
String sensorsname[3]={"Volume","Temperature","Humidity"};  //Названия датчиков
float sensors_delta[3]={0,0.5,0.5};  // Сдвиг интервала включения/выключения чтобы избежать дребизга
#define Vol 0
#define Tem 1
#define Hum 2
#define NOFSENSORS sizeof(sensors)/sizeof(sensors[0])
#include <DHT.h>      // подключаем библиотеку для датчика
DHT dht(tempr_btn, DHT11);

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

#include <CircularBuffer.hpp>

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

}
void dprintln(long s,byte log=1)
{
  dprintln(String(s),log);

}
void print_sensors()
{
  for(int i=0;i<NOFSENSORS;i++)
   {
    dprint(sensorsname[i]);dprint(":");
    dprintln(String(sensors[i]));
   }
}
void printDigits(int digits,byte log=1)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    dprint("0",log);
  dprint(digits,log);
}



void digitalClockDisplay(){
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
 

//---------------------------------------------------------------------------
 scheduler::scheduler(String _schedulername,String _taskname,int8_t _btn,int8_t _StHour,int8_t _StMin,int8_t _StDayofWeek,unsigned long _duration,unsigned long _interval,unsigned long _preexitdelay,float _startV,float _stopV,int8_t _orderV,float _startT,float _stopT,int8_t _orderT,float _startH,float _stopH,int8_t _orderH,byte _r0,byte _r1,byte _r2,byte _r3,byte _r4,byte _r5,byte _r6,byte _r7,byte _r8,byte _r9)
   :schedulername(_schedulername),taskname(_taskname),btn(_btn),StHour(_StHour),StMin(_StMin),StDayofWeek(_StDayofWeek),duration(_duration),interval(_interval),preexitdelay(_preexitdelay)
{
byte trelays[NOFRELAYS];
startD[0]=_startV;
stopD[0]=_stopV;
orderD[0]=_orderV;
startD[1]=_startT;
stopD[1]=_stopT;
orderD[1]=_orderT;
startD[2]=_startH;
stopD[2]=_stopH;
orderD[2]=_orderH;
aID=-1; 
finish=0;
trelays[0]=_r0;trelays[1]=_r1;trelays[2]=_r2;trelays[3]=_r3;trelays[4]=_r4;trelays[5]=_r5;trelays[6]=_r6;trelays[7]=_r7;trelays[8]=_r8;trelays[9]=_r9;
for(int i=0;i<NOFRELAYS;i++)
 {
  if(trelays[i]!=0)
   {
 //   relays[trelays[i]-1]=i+1;
 relays[i]=trelays[i];
   }
 }
 laststop=0;
}

#define NOFALARMS 5

#define v1_3bochki 1000000
#define v1_2bochki 1500000
#define v2_3bochki 2000000
#define NOFSCHEDULER sizeof(schedulers_arr)/sizeof(scheduler)
#define NOFTASKS sizeof(tasks_arr)/sizeof(task)


#ifdef GPSTRACKER
#include "buildTime.h"
#define baseH 19
#define baseM 00
//int btns[]={violet_btn,white_btn,blue_btn,green_btn,yellow_btn,orange_btn/*,water_btn*/};
//scheduler nme,task name, button, start Hour, Start Min,Start day of week (Sinday =1),Duration,pre exit delay,start volume,stop volume,side of interval,start tempr,stop tempr,sire of tempr interval,start hum,stop hum,side of hum interval,
//side of interval 1 - between start and stop 0 - outside start and stop
// последние 8 цифр это номера релле используемые в задаче в порядке их использования. Релле нумеруются от 1 !!!
 


//!!!! it's DEBUG - GPSTRACKER
scheduler schedulers_arr[]={ 
  {"Violet task",violet_btn,BUILD_HOUR,BUILD_MIN+6, 5,40000,0,2,1,10,300,1,0,110,1,1,7,8,0,0,0,0,0},
  {"White task",white_btn,BUILD_HOUR,BUILD_MIN+2,  5,30000,0,2,1,10,300,1,0,110,1,2,7,8,0,0,0,0,0},
  {"blue task",blue_btn,BUILD_HOUR,BUILD_MIN+3,  -1,15000,0,5,1,10,300,1,0,110,1,3,7,8,0,0,0,0,0},
  {"green task",green_btn,BUILD_HOUR,BUILD_MIN+4,  5,15000,0,3,1,10,300,1,0,110,1,4,7,8,0,0,0,0,0},
  {"yellow task",yellow_btn,BUILD_HOUR,BUILD_MIN+5,-1,15000,0,5,1,10,300,1,0,110,1,5,7,8,0,0,0,0,0},
   {"tempr task",-1,-1,-1,-1,500,0,500,1,30,300,0,0,110,1,6,0,0,0,0,0,0,0},
   {"Orange task",orange_btn,-1,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

 /* {"Orange task",-1,-1,-1,0,-1,0}/*,
  {"Water task",water_btn,orange_rel,-1,-1,-1,0,0,&stopInit,&stopStart,&stopExec,&stopFin}*/
  };
 
#else
// !!!! its RELEASE
scheduler schedulers_arr[]={
//  
/*
  {"Violet task",violet_btn,10,0,6,v1_2bochki,0,100,1,18,300,1,0,110,1},
  {"White task",white_btn,10,0,7,v1_2bochki,0,100,1,18,300,1,0,110,1},
  {"blue task",blue_btn,10,0,1,v1_2bochki,0,100,1,18,300,1,0,110,1},
  {"green task",green_btn,18,35,-1,v1_3bochki,0,60,1,18,300,1,0,110,1},
  {"yellow task",yellow_btn,18,00,-1,v1_3bochki,0,60,1,18,300,1,0,110,1},
*/
#define CMD_BTN 100
/*String _schedulername,        название расписание
String _taskname,               название задачи
int8_t _btn,                    кнопка запуска
int8_t _StHour,                 час старта
int8_t _StMin,                  минута старта
int8_t _StDayofWeek,            день недели старта (если -1 то каждый день) ?
unsigned long _duration,        время работы задачи
unsigned long _interval,        минимальный интервал между остановкой и запуском
unsigned long _preexitdelay,    задержка перед выходом?
float _startV,                  начало интервала значения датчика воды 
float _stopV,                   конец интервала значения датчика воды 
int8_t _orderV,                 задача работает пока значение объема воды внутри или вне интервала 0,1 соответсвенно
float _startT,
float _stopT,
int8_t _orderT,
float _startH,
float _stopH,
int8_t _orderH,
byte _r0,byte _r1,byte _r2,byte _r3,byte _r4,byte _r5,byte _r6,byte _r7   */
// relay 1,2 switch close, open
// 3,4,5 - open
//6,7,8 - close
  {"close all","pnevmo",orange_btn,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1, /*relays*/1,9,5,8,4,7,3,10,6,0},
  {"open E","pnevmo",white_btn,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/2,5,9,0,0,0,0,0,0,0},
  {"open W","pnevmo",blue_btn,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/2,8,4,0,0,0,0,0,0,0},
  {"open S","pnevmo",green_btn,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/2,7,3,0,0,0,0,0,0,0},
  {"open N","pnevmo",yellow_btn,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/2,10,6,0,0,0,0,0,0,0},
  {"close E","pnevmo",CMD_BTN,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/1,9,5,0,0,0,0,0,0,0},
  {"close W","pnevmo",CMD_BTN,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/1,8,4,0,0,0,0,0,0,0},
  {"close S","pnevmo",CMD_BTN,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/1,7,3,0,0,0,0,0,0,0},
  {"close N","pnevmo",CMD_BTN,-1,-1,-1,30000,0,0,/*volume*/-10,500,1,/*tempr*/-100,300,1,/*hym*/0,110,1,/*relays*/1,10,6,0,0,0,0,0,0,0},

  {"close auto E","pnevmo",-1,-1,-1,-1,15000,1800000,0,/*volume*/-10,500,1,/*tempr*/-100,20,1,/*hym*/0,110,1, /*relays*/1,9,5,0,0,0,0,0,0,0},
  {"open auto E","pnevmo",-1,-1,-1,-1,15000,1800000,0,/*volume*/-10,500,1,/*tempr*/26,300,1,/*hym*/0,110,1,/*relays*/2,5,9,0,0,0,0,0,0,0},
  {"close auto W","pnevmo",-1,-1,-1,-1,15000,1800000,0,/*volume*/-10,500,1,/*tempr*/-100,22,1,/*hym*/0,110,1, /*relays*/1,8,4,0,0,0,0,0,0,0},
  {"open auto W","pnevmo",-1,-1,-1,-1,15000,1800000,0,/*volume*/-10,500,1,/*tempr*/27,300,1,/*hym*/0,110,1,/*relays*/2,8,4,0,0,0,0,0,0,0},
  {"open auto S","pnevmo",-1,-1,-1,-1,20000,1800000,0,/*volume*/0.5,1.5,1,/*tempr*/25,300,1,/*hym*/0,110,1,/*relays*/2,7,3,0,0,0,0,0,0,0},
  {"close auto S","pnevmo",-1,-1,-1,-1,20000,1800000,0,/*volume*/-10,500,1,/*tempr*/-100,22,1,/*hym*/0,110,1, /*relays*/1,7,3,0,0,0,0,0,0,0},
  {"close rain S","pnevmo",-1,-1,-1,-1,20000,370000,0,/*volume*/-0.5,0.5,1,/*tempr*/20,40,1,/*hym*/0,110,1, /*relays*/1,7,3,0,0,0,0,0,0,0},
  {"open auto N","pnevmo",-1,-1,-1,-1,20000,1800000,0,/*volume*/0.5,1.5,1,/*tempr*/25,300,1,/*hym*/0,110,1,/*relays*/2,10,6,0,0,0,0,0,0,0},
  {"close auto N","pnevmo",-1,-1,-1,-1,20000,1800000,0,/*volume*/-10,500,1,/*tempr*/-100,22,1,/*hym*/0,110,1, /*relays*/1,10,6,0,0,0,0,0,0,0},
  {"close rain N","pnevmo",-1,-1,-1,-1,20000,370000,0,/*volume*/-0.5,0.5,1,/*tempr*/20,40,1,/*hym*/0,110,1, /*relays*/1,10,6,0,0,0,0,0,0,0}
  };
#endif





//---------------------------------------------------------------------------
// В новой редакции у task нет данных по релле и т.п. => инициализировать по сути нечего


// Стандартные процедуры start, exce, stop
class default_start: public general_start   
{
  public:
   virtual void start(proc *x) {
    for(int i=0;i<NOFRELAYS;i++)
     {
      if(x->currentShed->relays[i]>0)
       {
#ifdef DEBUG_PLUS
dprintln("try to start rellay"+String(x->currentShed->relays[i]));
#endif 
       digitalWrite(relays[x->currentShed->relays[i]-1],LOW);
       }
     }
   }
} defaultStart;

class default_exec: public general_do
{
  public:
   virtual void handle(proc *x) {

   }
} defaultExec;

class default_stop: public general_stop
{
  public:
   virtual void stop(proc *x) {

      for(int i=NOFRELAYS-1;i>=0;i--)   //отключаем реле в обратном порядке
        {
        if(x->currentShed->relays[i]!=0)
          digitalWrite(relays[x->currentShed->relays[i]-1],HIGH);
        }
    /* }  */
   }
} defaultStop;

// Start stop exec под пневмо цилиндры
#define PNEVMO_DELTA 100
class pnevmo_exec: public general_do
{
 public:
  long timer;
  bool startstop;
  bool openclose;
   virtual void handle(proc *x) {
    time_t ntime = now();
    if((hour(ntime)>7 and hour(ntime)<23))
     {
    if(timer<0)
    {
      startstop=0;
      timer=millis()+4*PNEVMO_DELTA;
    }
    else
     {
      if(timer<millis())
      {
      startstop=(!startstop);
      if(startstop==0)
       {
         timer=millis()+2*PNEVMO_DELTA;
       }
      else
       { 
         timer=millis()+10*PNEVMO_DELTA;
       }
      }
     }
 
   for(int i=0;i<NOFRELAYS;i++)
     {
      if(x->currentShed->relays[i]>2)
       {
       digitalWrite(relays[x->currentShed->relays[i]-1],startstop);
       }
     }
    }
  else
   {
     for(int i=0;i<NOFRELAYS;i++)
     {
       digitalWrite(relays[x->currentShed->relays[i]-1],LOW);
     }
   }
     
   }

} pnevmoExec;

class pnevmo_start: public general_start   
{
  public:
   virtual void start(proc *x) {
        if(x->currentShed->duration==PERMANENT_DURATION)
         {
         // Пневмо задача не может быть перманентной!!!  
         dprintln("Error!!! Permanent pnevmo task");
         return;
          
       //  x->finish=PERMANENT_DURATION;
         }
  pnevmo_exec * pne;
  pne=(pnevmo_exec*)x->currentTask->exec;   
  pne->openclose=0;
    for(int i=0;i<NOFRELAYS;i++)
     {
      if(x->currentShed->relays[i]>0) 
       {
        digitalWrite(relays[x->currentShed->relays[i]-1],LOW);
        if(x->currentShed->relays[i]==2)
         {
         pne->openclose=1;    //Если в списке есть реле - 2 значит выполняется открытие
         }
       }
     }

   pne->timer=-1;
   pne->startstop=HIGH;
   }
} pnevmoStart;


class pnevmo_stop: public general_stop
{
  public:
   virtual void stop(proc *x) {
       for(int i=NOFRELAYS-1;i>=0;i--)   //отключаем реле в обратном порядке
        {
          if(x->currentShed->relays[i]>2)
           {
           digitalWrite(relays[x->currentShed->relays[i]-1],HIGH);
           }
        }
          Alarm.delay(200);
          digitalWrite(relays[0],HIGH); //Насос и клапан отключаем отдельно принудительно
          digitalWrite(relays[1],HIGH); 
   }
} pnevmoStop;

// General_start / general_stop работает для всех task

void general_start::handle(proc *x)
{
   x->milstart=millis();  
  if(x->currentShed->duration==PERMANENT_DURATION)
    {
    x->finish=PERMANENT_DURATION;
    }
  else
    {
    x->finish=x->currentShed->duration+millis();
    }
  pulseCount=0;   //при старте типовой задачи сбрасываем датчик воды
     if(Alarm.count()<NOFALARMS)
      {
      x->currentShed->aID=-1;
#ifdef DEBUG_PLUS
int ia;
ia=getnextscheduler();
dprint("Next Alarm id: ");dprintln(ia);
bindShedulertoAlarm(ia);
dprint("Alarm count: ");dprintln(Alarm.count());
#else
      bindShedulertoAlarm(getnextscheduler());
#endif     
      }
  this->start(x);
  digitalClockDisplay();
  dprint(x->currentShed->schedulername);dprintln(" started");
#ifdef DEBUG_PLUS
dprintln(millis());
dprintln("x->finish set to"+String(x->finish));
dprintln("x->pnevmofinish set to"+String(x->finish));
#endif 

}

void general_stop::handle(proc *x)
{
    if(x->milstart!=0)
     {
      digitalClockDisplay();
      dprint(x->currentShed->schedulername);dprintln(" finished");
#ifdef DEBUG_PLUS
dprintln(millis());
dprint("real rain status:");
dprintln(digitalRead(water_btn));
#endif 
      print_sensors();
      x->currentShed->laststop=millis();
      this->stop(x);
     }


   x->freeproc();
}
//------------------------------------------------------------------------------


class stop_start: public general_start
{
  public:
   virtual void start(proc *x) {
    digitalClockDisplay();
    dprintln(" emergancy STOP");
    Alarm.delay(10000);
    emergency_stop();
   }
} stopStart;

class stop_exec: public general_do
{
  public:
   virtual void handle(proc *x) {

   }
} stopExec;

class stop_fin: public general_stop
{
  public:
   virtual void stop(proc *x) {

   }
} stopFin;




//----------------------------------------------------------------------------


proc::proc()
{
  interruped=0;
  aID=-1;
  milstart=0;
  finish=0;
  currentTask=NULL;
  currentShed=NULL;
}

void proc::freeproc()
{
  interruped=0;
  aID=-1;
  milstart=0;
  finish=0;
  currentTask=NULL;
  currentShed=NULL;
 
}
proc procs_arr[NOFPROCS];

int getfreeproc()  //ищем свободный процесс 
{
  int freeproc=-1;
  for(int i=0;i<NOFPROCS;i++)
   {
#ifdef DEBUG_PLUS   
dprintln("i:"+String(i)+" procs_arr[i].milstart:"+String(procs_arr[i].milstart));
#endif
    if(procs_arr[i].milstart==0)
     {
      freeproc=i;
      break;
     }
   }
   return(freeproc);
}



void proc::check()
{  //проверяем не закончилось ли время работы процесса
#ifdef DEBUG_LOOP
dprintln("next check.proc started");
#endif 

if(finish>0)
 {
#ifdef DEBUG_LOOP
dprintln("check.proc "+currentShed->schedulername+" finish ="+String(finish));
#endif 

    if(finish>millis())
     {
      currentTask->exec->handle(this);
#ifdef DEBUG_LOOP
dprintln("check.proc "+currentShed->schedulername+" exec done ");

#endif
     }
    else
     {
#ifdef DEBUG_LOOP
dprintln("check.proc "+currentShed->schedulername+" GO TO STOP ");

#endif
      currentTask->fin->handle(this);
     }
 } 
}


task::task(String _taskname,general_start * _start,general_do * _exec,general_stop * _fin)
   :taskname(_taskname),start(_start),exec(_exec),fin(_fin)
{
#ifdef DEBUG_PLUS
dprintln("--------------------init--------------"); 
#endif
}

task tasks_arr[]={
  {"pnevmo",&pnevmoStart,&pnevmoExec,&pnevmoStop},
 // {" close",&pnevmoStart,&pnevmoExec,&pnevmoStop},
  {"Orange task",&stopStart,&stopExec,&stopFin}
  };
 
int getnextscheduler()   //Ищем ближайший по времени scheduler 
{
  
int nextscheduler=-1;
int j=0;
time_t ntime = now();
time_t tt;
time_t ct=now()+8*SECS_PER_DAY;
  for (int i=0;i<NOFSCHEDULER;i++) 
  {
   if(schedulers_arr[i].StMin>=0 && schedulers_arr[i].StHour>=0) //Только если не перманентный scheduler
    {
    if((schedulers_arr[i].aID<0)  /*&& (tasks_arr[j].aID<0)  */)
     {
      if(schedulers_arr[i].StDayofWeek>0)
       {
        tt=previousSunday(ntime)+(schedulers_arr[i].StDayofWeek-1) * SECS_PER_DAY + AlarmHMS(schedulers_arr[i].StHour,schedulers_arr[i].StMin,0);
        if(tt<=ntime)
         {
          tt=nextSunday(ntime)+(schedulers_arr[i].StDayofWeek-1) * SECS_PER_DAY + AlarmHMS(schedulers_arr[i].StHour,schedulers_arr[i].StMin,0);
         }
       }
      else
       {
        tt=previousMidnight(ntime)+AlarmHMS(schedulers_arr[i].StHour,schedulers_arr[i].StMin,0);
                if(tt<=ntime)
         {
           tt=nextMidnight(ntime)+AlarmHMS(schedulers_arr[i].StHour,schedulers_arr[i].StMin,0);
         }
       }
      if(tt<ct)
       {
        ct=tt;
        nextscheduler=i;
       }
     }
    }
  }
#ifdef DEBUG_PLUS
dprint("got next scheduler");
dprintln(nextscheduler);
#endif    
  return(nextscheduler);
}
int startnewproc(scheduler *nsched)
{
  
int i;
int procid=-1;
            for(i=0;i<NOFPROCS;i++)
            {
               if(procs_arr[i].currentShed==nsched)  //есть процесс с этим scheduler 
                break;
            }
            if(i>=NOFPROCS)
             {
             procid=getfreeproc();
             if(procid>=0) //есть свободный процесс
              {
              procs_arr[procid].currentShed=nsched;
              for(task & ntask :tasks_arr)
               {
                if(ntask.taskname.equals(nsched->taskname))
                  {
                  procs_arr[procid].currentTask=&ntask;
                  break;
                  }
               }
              procs_arr[procid].milstart=millis();/*
              procs_arr[procid].finish=procs_arr[procid].milstart+nsched->duration;
              procs_arr[procid].aID= nsched->aID;*/
              procs_arr[procid].currentTask->start->handle(&procs_arr[procid]); 
              }
             else
              {
              dprint("No free proc for task:"+nsched->taskname+" scheduler id:"+String(i));
              }
             }
           else
            {
             dprint("scheduler id:"+String(id)+" already running");
             procid=i;
            }
return(procid);
}
void scheduler::check()
{

byte isrun=0;
int proc_id;
bool sens=true;
#ifdef DEBUG_LOOP
dprintln("check.Scheduler "+schedulername+" started");
#endif
for(proc_id=NOFPROCS-1;proc_id>=0;proc_id--)
 {
 if(procs_arr[proc_id].currentShed!=NULL)
  {
#ifdef DEBUG_LOOP
dprintln("procs_arr[proc_id].currentShed!=NULL procid="+String(proc_id));
#endif 

  if(procs_arr[proc_id].currentShed->schedulername==this->schedulername)  //есть процесс с этим scheduler 
    break;
  }
 }
#ifdef DEBUG_LOOP
dprintln("check.Scheduler "+schedulername+" proc seek finished "+String(proc_id));
#endif 
for(int i=0;i<NOFSENSORS;i++)
     {
      if(sensors[i]<-10000)
       sens*=1;
      else
       {
        
       float dd=0;
       if(proc_id<0)  //Если нет процесса с таким scheduler то двигаем диапазон на sensors_delta[i], чтобы избежать дребезга 
         {
         dd=sensors_delta[i]*((this->orderD[i])?1.:-1.);
//          dprint("dd:");dprint(String(dd));dprint(" sensor value  ");dprint(String(sensors[i]));dprint(" sensor delta  ");dprintln(String(sensors_delta[i]));
         }
        
         sens*=(this->startD[i]+dd<=sensors[i] && sensors[i]<=this->stopD[i]-dd)?this->orderD[i]:(!this->orderD[i]);
       
       }

     }
#ifdef DEBUG_LOOP
dprintln("check.Scheduler "+schedulername+" sensors checked sens:"+String(sens));
#endif
 
if(btn>0)
  {
  bool btnstatus=false;
  if(btn<40) 
   {
    btnstatus=digitalRead(btn);
   }
  if(!btnstatus) 
   {
    btcmd.trim();    
    btnstatus=schedulername.equals(btcmd);
 //   if(btcmd!="")
 //   dprint("btcmd="+btcmd+"btnstatus "+String(btnstatus));
  
   }
  if(btnstatus==HIGH && millis()>btnHIGHmillis+BTNTRIM)
   {
#ifdef DEBUG_PLUS
dprintln("----------------check.Scheduler "+schedulername+" button presed. ProcID"+String(proc_id));
#endif
    laststop=0;
    btnHIGHmillis=millis();
    if(proc_id<0)
     {
      sens=sens*1;
     }
    else
     {
      sens=false;
      procs_arr[proc_id].interruped=1;
     }
   }
  else
   {
      if(proc_id<0)
       {
          sens=false;
 //         btnHIGHmillis=0;
       }
      else
       {
        if(millis()> btnHIGHmillis+BTNTRIM+this->preexitdelay && procs_arr[proc_id].interruped==1)  
         {  //Кнопка задана, не нажата, есть процесс и запущен останов (было не завершенное нажатие) + у процесса стоит флаг interruped
           procs_arr[proc_id].interruped=0;
           sens=false;
         }
         
       }
   }
  }

 if(sens)
     {
#ifdef DEBUG_LOOP
dprintln("Scheduler "+schedulername+" sens true");
#endif
     if(proc_id<0 && this->StMin<0)  //Если расписание не задано - стартуем. Если у задачи есть расписание, то она стартует ТОЛЬКО по расписанию 
      {                               // Если нужен ручной старт то надо создать еще один scheduler без расписания
      if((laststop==0) || ((laststop+interval)<millis()))
      startnewproc(this);
      }
     }
    else
     {
#ifdef DEBUG_LOOP
dprintln("Scheduler "+schedulername+" sens FALSE");
#endif
     if(proc_id>=0)   //Если есть процесс этого scheduler
      {
      if(this->StMin<0) //если это перманентная задача
       {
           procs_arr[proc_id].finish=millis()+this->preexitdelay;  
       }

      if( (procs_arr[proc_id].finish<millis() && this->StMin<0)|| this->StMin>=0) // если вышло время задержки или задача не перманентная - выключаем реле
       {                                                                                           // в порядке обратном включению !!!! 
           procs_arr[proc_id].finish=0;
           procs_arr[proc_id].currentTask->fin->handle(&procs_arr[proc_id]);
       }
      }  
     }
#ifdef DEBUG_LOOP
dprintln("Scheduler "+schedulername+" check finised");
#endif
}

int bindShedulertoAlarm(int id)
{
  
if(id>=0)
 {
  int aID=-1;
           if(schedulers_arr[id].StDayofWeek>0)
           {
            aID=Alarm.alarmOnce((timeDayOfWeek_t)schedulers_arr[id].StDayofWeek,schedulers_arr[id].StHour,schedulers_arr[id].StMin,0,doalarm);
          }
          else
          {
          aID=Alarm.alarmOnce(schedulers_arr[id].StHour,schedulers_arr[id].StMin,0,doalarm);
          }
#ifdef DEBUG_PLUS
dprint("Set scheduler:");
dprint(id );
dprint(" task:");
dprint(schedulers_arr[id].taskname);
dprint(" on:");
digitalClockDisplay(Alarm.getNextTrigger(aID));
#endif     
        schedulers_arr[id].aID=aID;

   return(aID);
 }
else
 {
  return(-1);
 }
}

#define USE_SPECIALIST_METHODS
void emergency_stop()
{
  
//need revision !!!

    for (proc & nproc : procs_arr) 
     {
     nproc.finish=0;
     nproc.currentTask->fin->handle(&nproc);
     }

     for (proc & nproc : procs_arr) 
     {
     if(nproc.aID>=0)
      {
     dprint(nproc.aID);dprint(" "); dprint( nproc.currentTask->taskname);dprint(" ");dprint(Alarm.read(nproc.aID));dprint(" ");
     digitalClockDisplay(Alarm.read(nproc.aID),1);
      }
     }
  setTimefromGPS(60000);
  mill_restart=millis()+1000*300;  //автоматический перезапуск через 5 минут или по окончании работающего задания 
}

void doalarm()
    {
     int id;
      id=Alarm.getTriggeredAlarmId();
      int i;
       for (scheduler & nsched : schedulers_arr) 
        {
          if(nsched.aID==id)  
           {
            startnewproc(&nsched);
            break;

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
#define RAINDELAY 120000
unsigned long rainstart=0;
void refresh_sensors()
{
  //sensors[Vol]=pulseCount*2.25/1000.;


  if(digitalRead(water_btn)==0)
   {
#ifdef DEBUG_LOOP
   dprintln("rain sensor is 0");

#endif
    rainstart=millis();
    sensors[Vol]=0;  // Датчик дождя
   }
   else
   {
    if(((rainstart+RAINDELAY)>millis()) && (rainstart>0))
     {
      sensors[Vol]=0;
     }
     else
     {
      sensors[Vol]=1;
     }
   }

  
  float hp = dht.readHumidity();
  float tp = dht.readTemperature();
  if(!isnan(hp)) sensors[Hum]=hp;
  if(!isnan(tp)) sensors[Tem]=tp;
}


void setup() {
Serial.begin(115200);
/*File file = SPIFFS.open(CONFIGFILE, FILE_READ);
if(file)
{
  
}*/

SerialBT.begin("provetrivanie");
dht.begin();
SerialBT.register_callback(btCallback);
int i;
for (i=0;i<NOFRELAYS;i++)
 {
  pinMode(relays[i], OUTPUT);
  digitalWrite(relays[i], HIGH);
 }
for(i=0;i<NOFBTNS;i++)
 {
  pinMode(btns[i], INPUT);
 }

   Serial1.begin(9600, SERIAL_8N1, GPS_PIN_TX, GPS_PIN_RX);
   Serial1.setRxBufferSize(1024);
   Alarm.delay(500);  
   dprintln("Search GPS..");
   Alarm.delay(1000);
//Подключаем датчик дождя
pinMode(water_btn, INPUT_PULLUP);
digitalRead(water_btn);
//attachInterrupt(digitalPinToInterrupt(water_btn), pulseCounter, FALLING);

refresh_sensors();
setTime(setTimefromGPS(60*30*1000));
setSyncProvider(CsetTimefromGPS);
setSyncInterval(TIME_UPDATE);
// Устанавливаем таймеры
for(int i=0;i<NOFALARMS;i++)
 {
  bindShedulertoAlarm(getnextscheduler());
 }

#ifdef DEBUG_PLUS
dprintln("Procs compleate");
#endif 

//Сбрасываем значения датчиков в неопределено 
 for(int i=0;i<NOFSENSORS;i++)
  {
    sensors[i]=-10001;
  }
  dprintln("Setup complite");
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
 print_sensors();
#ifdef NOGPSDEBUG
 tm.Year=nogpsdate[nogps_i][5]-1970;
 tm.Month=nogpsdate[nogps_i][4];
 tm.Day=nogpsdate[nogps_i][3];
 tm.Hour=nogpsdate[nogps_i][0];
 tm.Minute=nogpsdate[nogps_i][1];
 tm.Second=nogpsdate[nogps_i][2];
  tt=makeTime(tm)+(offset * SECS_PER_HOUR);
  nogps_i++;
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
 tm.Year=gps.date.year()-1970;
 tm.Month=gps.date.month();
 tm.Day=gps.date.day();
 tm.Hour=gps.time.hour();
 tm.Minute=gps.time.minute();
 tm.Second=gps.time.second();
 //age=gps.time.age();
 tt=makeTime(tm)+offset * SECS_PER_HOUR;;
 dprint("Got time from GPS ");
 #endif
  digitalClockDisplay(tt);
return(tt);

}

 
void loop() {
  int i;
  Alarm.delay(10);
  if (Serial1.available()) {  //непрерывно читаем GPS 
   char g;
   g = Serial1.read();
#ifdef    DEBUG_GPS
    Serial.print(g);
#endif
   gps.encode(g);
   }

#ifdef DEBUG_LOOP
 dprintln("gps decoded");
#endif
refresh_sensors();
#ifdef DEBUG_LOOP
 dprintln("Sensors checked");
#endif
 for (proc & nproc : procs_arr) nproc.check();  // непрерывно запускаем check по всем процессам
  #ifdef DEBUG_LOOP
 dprintln("Proc.check() complit");
 #endif

 for (scheduler & nscheduler : schedulers_arr) nscheduler.check();  // непрерывно запускаем check по всему рассписанию
 #ifdef DEBUG_LOOP
 dprintln("shheduler check complit");
 #endif

 if(pushbuffer==1)   //при подключении bluetooth Serial выводим в порт log, расписание и т.п.
  {
  pushbuffer=0;
  using index_t = decltype(buffer)::index_t;
  for (index_t i = 0; i < buffer.size()-2; i++) {
     SerialBT.print((char)buffer[i]);
     }

//   float hp = dht.readHumidity();
//   float tp = dht.readTemperature();
   dprint("real rain status:"); dprintln(digitalRead(water_btn));
   dprint("Volume:");  dprint(String(sensors[Vol]));
   dprint("Humidity:");  dprint(String(sensors[Hum]));
   dprint("Temperature:");  dprintln(String(sensors[Tem]));
   }
btcmd="";
  if(SerialBT.available())
    {
       btcmd=SerialBT.readString();
    }

if(millis()>mill_restart) 
 {
  byte rst=1;
   for (proc & nproc : procs_arr) 
    {
    if(nproc.finish>millis() && nproc.currentShed->StMin>=0) rst=0;   
    }
  if(rst) ESP.restart();  //если вдруг все долго работает без перезагрузок, и нет запущенных задач то лучше бы перезагрузиться
 }
}
