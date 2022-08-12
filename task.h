
class general_do;
class task
{
  public:
  task(String,int,int,general_do *,general_do *,general_do *,general_do *);
 /* void init();
  void start();
  void exec();
  void fin();*/
  void check();
  String taskname;
  int btn;
  int relay;
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
  byte stat;
  int aID;
  int schedulerID;
};
class scheduler
{
   public:
  scheduler(String,int8_t,int8_t,int8_t,unsigned long,float,float,float,byte);
  String taskname;
  int8_t StHour;
  int8_t StMin;
  int8_t StDayofWeek;
  unsigned long duration;
  float volume;
  float startT;
  float stopT;
  byte orderT;
  unsigned long finish;
 // byte stat;
  int aID;
};
