class general_do;
class general_start;
class general_stop;
class scheduler   // содержит в себе все условия запуска и остановки задачи taskname
{
   public:
// scheduler::scheduler(String _schedulername,String _taskname,int8_t _btn,int8_t _StHour,int8_t _StMin,int8_t _StDayofWeek,unsigned long _duration,unsigned long _interval,unsigned long _preexitdelay,float _startV,float _stopV,int8_t _orderV,float _startT,float _stopT,int8_t _orderT,float _startH,float _stopH,int8_t _orderH,byte _r0,byte _r1,byte _r2,byte _r3,byte _r4,byte _r5,byte _r6,byte _r7)
//   :schedulername(_schedulername),taskname(_taskname),btn(_btn),StHour(_StHour),StMin(_StMin),StDayofWeek(_StDayofWeek),duration(_duration),interval(_interval),preexitdelay(_preexitdelay)

  scheduler(String,String,int8_t,int8_t,int8_t,int8_t,unsigned long,unsigned long,unsigned long,float,float,int8_t,float,float,int8_t,float,float,int8_t,byte,byte,byte,byte,byte,byte,byte,byte,byte,byte);
  String schedulername;
  String taskname;
  int8_t StHour;
  int8_t StMin;
  int8_t StDayofWeek;
  unsigned long duration;
  unsigned long interval;
  unsigned long preexitdelay;
  int8_t btn; 
  unsigned long btnHIGHmillis=0; 
  float startD[3];
  float stopD[3];
  byte orderD[3];
    void check();
   byte relays[10];
/*  float startT;
  float stopT;
  byte orderT;*/
  unsigned long laststop;
  unsigned long finish;
 // byte stat;
  int aID;
};



class task    // просто связка имени и трех функций start, exec , stop
{
  public:
  task(String,general_start *,general_do *,general_stop *);
 /* void init();
  void start();
  void exec();
  void fin();*/
  
  String taskname;
//  general_do * init;
  general_start * start;
  general_do * exec;
  general_stop * fin;

//  int schedulerID;

};

class proc    // Объект для запущеной задачи от конкретного scheduler
{
  public:
  proc();
  task *currentTask;
  scheduler *currentShed;
  byte interruped;
  int aID;
  unsigned long milstart;
  unsigned long finish;
  void check();
   void freeproc();
};
