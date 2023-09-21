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
#define NOFRELAYS sizeof(relays)/sizeof(relays[0])
#define NOFPROCS 10

unsigned long mill_restart=400000000;   //интервал автоматической перезагрузки
int relays[]={violet_rel,white_rel,blue_rel,green_rel,yellow_rel,orange_rel,brown_rel,lightgreen_rel};
#define NOFBTNS sizeof(btns)/sizeof(btns[0])
int btns[]={violet_btn,white_btn,blue_btn,green_btn,yellow_btn,orange_btn/*,water_btn*/};
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
scheduler::scheduler(String _taskname,int8_t _btn,int8_t _StHour,int8_t _StMin,int8_t _StDayofWeek,unsigned long _duration,float _startV,float _stopV,int8_t _orderV,float _startT,float _stopT,int8_t _orderT,float _startH,float _stopH,int8_t _orderH,byte _r0,byte _r1,byte _r2,byte _r3,byte _r4,byte _r5,byte _r6,byte _r7)
   :taskname(_taskname),btn(_btn),StHour(_StHour),StMin(_StMin),StDayofWeek(_StDayofWeek),duration(_duration)
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
trelays[0]=_r0;trelays[1]=_r1;trelays[2]=_r2;trelays[3]=_r3;trelays[4]=_r4;trelays[5]=_r5;trelays[6]=_r6;trelays[7]=_r7;
for(int i=0;i<NOFRELAYS;i++)
 {
  if(trelays[i]!=0)
   {
    relays[trelays[i]]=i;
   }
 }
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
//task name, button, start Hour, Start Min,Start day of week (Sinday =1),Duration,start volume,stop volume,side of interval,start tempr,stop tempr,sire of tempr interval,start hum,stop hum,side of hum interval,
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
  
/*
  {"Violet task",violet_btn,10,0,6,v1_2bochki,0,100,1,18,300,1,0,110,1},
  {"White task",white_btn,10,0,7,v1_2bochki,0,100,1,18,300,1,0,110,1},
  {"blue task",blue_btn,10,0,1,v1_2bochki,0,100,1,18,300,1,0,110,1},
  {"green task",green_btn,18,35,-1,v1_3bochki,0,60,1,18,300,1,0,110,1},
  {"yellow task",yellow_btn,18,00,-1,v1_3bochki,0,60,1,18,300,1,0,110,1},
*/
  {"teplica close",white_btn,-1,-1,-1,30000,0,500,1,20,300,0,0,110,1,7,1,2,0,0,0,0,0},
  {"Orange task",orange_btn,-1,0,-1,0,0,0,0,0,0,0,0,0,0,7,1,2,0,0,0,0,0}/*,
  {"Water task",water_btn,orange_rel,-1,-1,-1,0,0,&stopInit,&stopStart,&stopExec,&stopFin}*/
  };
#endif





//---------------------------------------------------------------------------
// В новой редакции у task нет данных по релле и т.п. => инициализировать по сути нечего
/*
class general_init: public general_do
{
  public:

 
   virtual void handle(task *x) {
#ifdef DEBUG_PLUS
    digitalClockDisplay();
    dprint (x->taskname);dprintln(" init");
#endif
for(int i=0;i<NOFRELAYS;i++)
 {
   if(x->relays[i]>0)
    pinMode(relays[x->relays[i]], OUTPUT);
    digitalWrite(relays[x->relays[i]], HIGH);
 }
   }
} generalInit;

*/
#ifdef DEBUG_VIOLET_BTN
int vts=0;
#endif
class default_start: public general_start   
{
  public:
   virtual void start(proc *x) {
    digitalClockDisplay();
    dprint(x->currentTask->taskname);dprintln(" started");
 
     if(x->currentShed->StMin>=0)  //если задача перманентная то задержка 49 дней (раньше сработает штатная перезагрузка)
     {
        x->finish=x->currentShed->duration+millis();
        pulseCount=0;   //при старте типовой задачи сбрасываем датчик воды

     }
       else//если задача перманентная то задержка 49 дней (раньше сработает штатная перезагрузка)
        {
        x->finish=4294967290;
        }
#ifdef DEBUG_PLUS
dprintln("x->finish set to"+String(x->finish));
#endif 
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

bool sens=true;
    for(int i=0;i<NOFSENSORS;i++)
     {
      if(sensors[i]<-10000)
       sens*=1;
      else
       {
        
       float dd=0;
       if(x->interruped!=0)  //Если задача прервана то двигаем диапазон на sensors_delta[i], чтобы избежать дребезга 
         {
         dd=sensors_delta[i]*((x->currentShed->orderD[i])?1.:-1.);
//          dprint("dd:");dprint(String(dd));dprint(" sensor value  ");dprint(String(sensors[i]));dprint(" sensor delta  ");dprintln(String(sensors_delta[i]));
         }
        
         sens*=(x->currentShed->startD[i]+dd<=sensors[i] && sensors[i]<=x->currentShed->stopD[i]-dd)?x->currentShed->orderD[i]:(!x->currentShed->orderD[i]);
       
       }
#ifdef DEBUG_MINUS   
     dprint(x->taskname);dprint(" sens ");dprint(i); dprint(" value");dprint(sens);dprint(" sensor value  ");dprintln(sensors[i]);
#endif   
     }
    if(sens)
     {
     for(int i=0;i<NOFRELAYS;i++)
     {
      if(x->currentShed->relays[i]!=0)
        digitalWrite(relays[x->currentShed->relays[i]-1],LOW);
     }
     if(x->interruped!=0)
      {
       digitalClockDisplay();
       dprint(x->currentShed->taskname);dprintln(" resume"); 
       dprint("sensors: ");
       print_sensors();
      x->interruped=0;
      }
     }
    else
     {
      if(x->currentShed->StMin<0) //если это перманентная задача
       {
        
       
        if(x->currentShed->finish==0 && (!x->interruped))
         {                                                            //если это перманентная задача
          x->currentShed->finish=millis()+x->currentShed->duration;  //то duration это задержка отключения после выхода датчика из диапазона
         }
       }

     //  Serial.println(schedulers_arr[x->schedulerID].finish-millis());
      if( (x->currentShed->finish<millis() && x->currentShed->StMin<0)|| x->currentShed->StMin>=0) // если вышло время задержки или задача не перманентная - выключаем реле
       {                                                                                           // в порядке указанном при инициализации задачи
       for(int i=0;i<NOFRELAYS;i++)
        {
        if(x->currentShed->relays[i]!=0)
        digitalWrite(relays[x->currentShed->relays[i]-1],HIGH);
        }
       if(!x->interruped)
        {
       digitalClockDisplay();
       dprint(x->currentTask->taskname);dprintln(" interruped"); 
       dprint("sensors: ");
       print_sensors();
      x->interruped=1;
       }
       if(x->currentShed->StMin<0) // Если задача перманентная сбрасываем задержку
        {
           x->currentShed->finish=0;
        }
      }
     }
   }
} defaultExec;

class default_stop: public general_stop
{
  public:
   virtual void stop(proc *x) {
    if(x->milstart!=0)
     {
#ifdef DEBUG_VIOLET_BTN
   if(x->currentShed->taskname.equals("Violet task"))
    {
      vts=0;      
    }
#endif
      digitalClockDisplay();
      dprint(x->currentShed->taskname);dprintln(" stoped");
      print_sensors();
      for(int i=0;i<NOFRELAYS;i++)
        {
        if(x->currentShed->relays[i]!=0)
          digitalWrite(relays[x->currentShed->relays[i]-1],HIGH);
        }
     }
   }
} defaultStop;

void general_start::handle(proc *x)
{
  this->start(x);
  x->milstart=millis();  
#ifdef DEBUG_VIOLET_BTN
dprint("Alarm count: ");dprintln(Alarm.count());
#endif  
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
}

void general_stop::handle(proc *x)
{
 
   this->stop(x);
   x->currentShed=NULL;
   x->aID=-1;
   x->milstart=0;
}
//------------------------------------------------------------------------------
/*class stop_init: public general_do
{
  public:
   virtual void handle(task *x) {
  
   }
} stopInit;*/

class stop_start: public general_start
{
  public:
   virtual void start(proc *x) {
    digitalClockDisplay();
    dprintln(" emergancy STOP");
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

task::task(String _taskname,general_start * _start,general_do * _exec,general_stop * _fin)
   :taskname(_taskname),start(_start),exec(_exec),fin(_fin)
{
#ifdef DEBUG_PLUS
dprintln("--------------------init--------------"); 
#endif
}

void proc::check()
{  //проверяем не закончилось ли время работы процесса
if(finish>0)
 {
    if(finish>millis())
     {
      currentTask->exec->handle(this);
#ifdef DEBUG_LOOP
dprintln("exec done");
#endif
     }
    else
     {
      currentTask->fin->handle(this);
     }
 } 
}
//int relays[]={violet_rel,white_rel,blue_rel,green_rel,yellow_rel,orange_rel,brown_rel,lightgreen_rel};


task tasks_arr[]={
  {"Violet task",&defaultStart,&defaultExec,&defaultStop},
  {"teplica close",&defaultStart,&defaultExec,&defaultStop},
  {"blue task",&defaultStart,&defaultExec,&defaultStop},
  {"green task",&defaultStart,&defaultExec,&defaultStop},
  {"yellow task",&defaultStart,&defaultExec,&defaultStop},
  {"Orange task",&stopStart,&stopExec,&stopFin}
  ,
  {"tempr task",&defaultStart,&defaultExec,&defaultStop}
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
 /* for (j=0;j<NOFTASKS;j++)
   {
    if(tasks_arr[j].taskname.equals(schedulers_arr[i].taskname)) 
     break;
   }
 */  
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
              procs_arr[procid].milstart=millis();
              procs_arr[procid].currentTask->start->handle(&procs_arr[procid]); 
#ifdef DEBUG_PLUS
dprintln("New proc started"+String(procid));
#endif              
              }
             else
              {
              dprint("No free proc for task:"+nsched->taskname+" scheduler id:"+String(i));
              }
  //           ntask.start->handle(&ntask);
//            break;
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
  
// Need revision!!!

  if(btn>0)
  {
#ifdef DEBUG_VIOLET_BTN
#ifdef GPSTRACKER
   if(btn==violet_btn && vts==0)
    {
      vts++;
#else 
   if(digitalRead(btn)==HIGH && millis()>btnHIGHmillis+BTNTRIM)
   {
#endif
#else
  if(digitalRead(btn)==HIGH && millis()>btnHIGHmillis+BTNTRIM)
   {
#endif
    byte isrun=0;
    btnHIGHmillis=millis();
    for (proc & nproc : procs_arr)
     {
       if(this==nproc.currentShed)  //если есть процесс с этим scheduler который то останавливаем процесс
        {
        isrun++;
        nproc.currentTask->fin->handle(&nproc);
        }
     }
   // Если все процессы перебрали и не нашли запускаем новый
    if(isrun==0)
     {
      startnewproc(this);
     }
   }
  }
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

void refresh_sensors()
{
  sensors[Vol]=pulseCount*2.25/1000.;
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
#ifdef GPSTRACKER
// Если для отладки использую плату трекера
 pinMode(23, OUTPUT);
 digitalWrite(GPIO_NUM_23, HIGH);
#endif

refresh_sensors();
setTime(setTimefromGPS(60*30*1000));
setSyncProvider(CsetTimefromGPS);
setSyncInterval(TIME_UPDATE);
// Иницализируем задачи
// Устарело for (task & ntask : tasks_arr) ntask.init->handle(&ntask);
// Устанавливаем таймеры
for(int i=0;i<NOFALARMS;i++)
 {
  bindShedulertoAlarm(getnextscheduler());
 }
for (i=0;i<NOFSCHEDULER;i++) 
 {
  if(schedulers_arr[i].StMin<0 && schedulers_arr[i].StHour<0)  // ищем и запускаем перманентные задачи
   {
   startnewproc(&schedulers_arr[i]);
   }
 }
#ifdef DEBUG_PLUS
dprintln("Procs compleate");
#endif 
//Подключаем расходомер
pinMode(water_btn, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(water_btn), pulseCounter, FALLING);
#ifdef DEBUG_PLUS
dprintln("rashod attached");
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
   dprint("Volume:");  dprint(String(sensors[Vol]));
   dprint("Humidity:");  dprint(String(sensors[Hum]));
   dprint("Temperature:");  dprintln(String(sensors[Tem]));
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
