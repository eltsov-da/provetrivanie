#include "task.h"
class general_do {
  public:
    virtual void handle(proc *) = 0;
//  virtual void handle() = 0;
};
class general_stop : public general_do  {
  public:
   void handle(proc *);
   virtual void stop(proc *) = 0;
//  virtual void handle() = 0;
};
class general_start : public general_do  {
  public:
   void handle(proc *);
   virtual void start(proc *) = 0;
//  virtual void handle() = 0;
};
