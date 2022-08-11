
class general_do;
class task
{
  public:
  task(String,int,int,int8_t,int8_t,int8_t,unsigned long,unsigned long,general_do *,general_do *,general_do *,general_do *);
 /* void init();
  void start();
  void exec();
  void fin();*/
  void check();
  String taskname;
  int btn;
  int relay;
  int8_t StHour;
  int8_t StMin;
  int8_t StDayofWeek;
  unsigned long duration;
  unsigned long finish;
  general_do * init;
  general_do * start;
  general_do * exec;
  general_do * fin;
  byte stat;
  int aID;
};
