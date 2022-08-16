
class general_do;
class task
{
  public:
  task(String,int,general_do *,general_do *,general_do *,general_do *,byte,byte,byte,byte,byte,byte,byte,byte);
 /* void init();
  void start();
  void exec();
  void fin();*/
  void check();
  String taskname;
  int btn;
//  int relay;
  byte relays[8];
/*  int8_t StHour;
  int8_t StMin;
  int8_t StDayofWeek;
  unsigned long duration;
  float volume;
  float startT;
  float stopT;
  byte orderT;*/
  unsigned long finish;
  general_do * init;
  general_do * start;
  general_do * exec;
  general_do * fin;
  byte interruped;
  byte stat;
  int aID;
  int schedulerID;
};
class scheduler
{
   public:
  scheduler(String,int8_t,int8_t,int8_t,unsigned long,float,float,byte,float,float,byte,float,float,byte);
  String taskname;
  int8_t StHour;
  int8_t StMin;
  int8_t StDayofWeek;
  unsigned long duration;
  // 
  float startD[3];
  float stopD[3];
  byte orderD[3];
/*  float startT;
  float stopT;
  byte orderT;*/
  unsigned long finish;
 // byte stat;
  int aID;
};
