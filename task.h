class general_do;
class general_start;
class general_stop;
class scheduler   // содержит в себе все условия запуска и остановки задачи taskname
{
   public:
  scheduler(String,int8_t,int8_t,int8_t,int8_t,unsigned long,float,float,int8_t,float,float,int8_t,float,float,int8_t,byte,byte,byte,byte,byte,byte,byte,byte);
  String taskname;
  int8_t StHour;
  int8_t StMin;
  int8_t StDayofWeek;
  unsigned long duration;
  int8_t btn; 
  unsigned long btnHIGHmillis=0; 
  float startD[3];
  float stopD[3];
  byte orderD[3];
    void check();
   byte relays[8];
/*  float startT;
  float stopT;
  byte orderT;*/
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
};
