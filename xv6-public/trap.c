#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"

// Interrupt descriptor table (shared by all CPUs).
struct gatedesc idt[256];
extern uint vectors[];  // in vectors.S: array of 256 entry pointers
struct spinlock tickslock;
uint ticks;

void
tvinit(void)
{
  int i;

  for(i = 0; i < 256; i++)
    SETGATE(idt[i], 0, SEG_KCODE<<3, vectors[i], 0);
  SETGATE(idt[T_SYSCALL], 1, SEG_KCODE<<3, vectors[T_SYSCALL], DPL_USER);
  SETGATE(idt[T_PROJECT01_2], 1 ,SEG_KCODE<<3, vectors[T_PROJECT01_2],DPL_USER);

  initlock(&tickslock, "time");
}

void
idtinit(void)
{
  lidt(idt, sizeof(idt));
}

//PAGEBREAK: 41
void
trap(struct trapframe *tf)
{
  if(tf->trapno == T_SYSCALL){
    if(myproc()->killed)
      exit();
    myproc()->tf = tf;
    syscall();
    if(myproc()->killed)
      exit();
    return;
  }

  if(tf->trapno == T_PROJECT01_2){
    cprintf("user interrupt %d called!\n", tf->trapno);
    return;
  }
  

  switch(tf->trapno){
  case T_IRQ0 + IRQ_TIMER:
    if(cpuid() == 0){
      acquire(&tickslock);
      ticks++;
      wakeup(&ticks);
      release(&tickslock);
    }
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE:
    ideintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE+1:
    // Bochs generates spurious IDE1 interrupts.
    break;
  case T_IRQ0 + IRQ_KBD:
    kbdintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_COM1:
    uartintr();
    lapiceoi();
    break;
  case T_IRQ0 + 7:
  case T_IRQ0 + IRQ_SPURIOUS:
    cprintf("cpu%d: spurious interrupt at %x:%x\n",
            cpuid(), tf->cs, tf->eip);
    lapiceoi();
    break;

  //PAGEBREAK: 13
  default:
    if(myproc() == 0 || (tf->cs&3) == 0){
      // In kernel, it must be our mistake.
      cprintf("unexpected trap %d from cpu %d eip %x (cr2=0x%x)\n",
              tf->trapno, cpuid(), tf->eip, rcr2());
      panic("trap");
    }
    // In user space, assume process misbehaved.
    cprintf("pid %d %s: trap %d err %d on cpu %d "
            "eip 0x%x addr 0x%x--kill proc\n",
            myproc()->pid, myproc()->name, tf->trapno,
            tf->err, cpuid(), tf->eip, rcr2());
    myproc()->killed = 1;
  }

  // Force process exit if it has been killed and is in user space.
  // (If it is still executing in the kernel, let it keep running
  // until it gets to the regular system call return.)
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();

  // Force process to give up CPU on clock tick.
  // If interrupts were on while locks held, would need to check nlock.
  #ifdef RR_SCHED
  if(myproc() && myproc()->state == RUNNING &&
     tf->trapno == T_IRQ0+IRQ_TIMER)
    yield();
  #elif FCFS_SCHED
  // if the SCHED_POLICY is FCFS_SCHED, current process doesn't yield CPU to some other process
  // but if process run more than 200ticks than kill the process
  if(myproc() && myproc()->state == RUNNING &&
     tf->trapno==T_IRQ0+IRQ_TIMER && (ticks - myproc()->allocated_time) >= 200){
    cprintf("200ticks over... kill %d process\n",myproc()->pid);
    kill(myproc()->pid);
  }
  #elif MULTILEVEL_SCHED
  // if the SCHED_POLICY is MULTILEVEL_SCHED, if pid is even use RR way, if pid is odd use FCFS way
  if(myproc() && ((myproc()->pid)%2==0) && myproc()->state == RUNNING && 
     tf->trapno==T_IRQ0+IRQ_TIMER ){
    yield();
  }else if(myproc() && (myproc()->pid)%2==1){
    if(myproc()->state==RUNNING && tf->trapno==T_IRQ0+IRQ_TIMER &&
     (ticks - myproc()->allocated_time) >= 200){
      cprintf("200ticks over... kill %d process\n", myproc()->pid);
      kill(myproc()->pid);
    }
  }
  #elif MLFQ_SCHED
  //this part is for L0 queue RR scheduling
  //if process state is running and is in L0 and not monopolize mode
  //if monopolized mode==1 than don't send time interrupt
  // but if it use all L0 time quantum- 4 ticks -then change level 0->1 and yield the process
  if(myproc()&&myproc()->state==RUNNING && myproc()->lev==0 &&
     myproc()->check_monopolize!=1 && (ticks-myproc()->allocated_time >=4)){
    changelev();
    yield();
  }
  //this part is for L1 queue priority scheduling
  //if process state is running and is in L1 and not monopolize mode
  // but if it us all L1 time quantum- 8 ticks - then reduce priority and re scheduling
  if(myproc()&&myproc()->state==RUNNING && myproc()->lev==1 &&
     myproc()->check_monopolize!=1 && (ticks-myproc()->allocated_time >=8)){
    changepriority();
    yield();
  }
  if(ticks %200==0)
    priorityboost();
  #endif

  // Check if the process has been killed since we yielded
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();
}
