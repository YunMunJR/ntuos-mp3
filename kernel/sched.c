#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"

extern struct proc proc[];

// ─────────────────────────────────────────────────────────────
// Scheduling algorithms
//
// Each algorithm implements:
//   struct sched_result schedule_<algo>(void)
//
// Returns the best RUNNABLE RT process and its computed
// allocated_time.  If no RT task is runnable, .process is 0.
// If a deadline miss is detected (RUNNABLE task with ticks > deadline),
// returns that task with allocated_time = 0 (miss-in-dispatch signal).
// The caller handles locking, dispatch logging, and swtch.
// ─────────────────────────────────────────────────────────────

struct sched_result
schedule_default(void)
{
  struct sched_result result = {0, 0};

  for(struct proc *p = proc; p < &proc[NPROC]; p++){
    if(p->state == RUNNABLE && p->is_real_time){
      result.process = p;
      if(p->current_exec_ticks < p->exec_time &&
         (int)ticks >= p->next_deadline){
        result.allocated_time = 0;
      }else{
        result.allocated_time = p->exec_time - p->current_exec_ticks;
      }
      return result;
    }
  }

  return result;
}

struct sched_result
schedule_efdf(void)
{
  // TODO, mp3 part 2, EFDF
  struct sched_result result = {0, 0};
  struct proc *feasible = 0, *infeasible = 0, *missed = 0;

  for(struct proc *p = proc; p < &proc[NPROC]; p++){
    if(p->state == RUNNABLE && p->is_real_time){
      int remaining = p->exec_time - p->current_exec_ticks;

      if((int)ticks >= p->next_deadline && remaining > 0){     // missed
        if(missed == 0){
          missed = p;
        }else{
          if(p->pid < missed->pid){
            missed = p;
          }
        }
        continue;
      }

      int deadline_delta = p->next_deadline - (int)ticks;
      int laxity = deadline_delta - remaining;
      
      if(laxity >= 0){  // feasible
        if(feasible == 0){
          feasible = p;
        }else{
          if(p->next_deadline < feasible->next_deadline)
            feasible = p;

          if(p->next_deadline == feasible->next_deadline && p->pid < feasible->pid)
            feasible = p;      
        }
        
      }else{   // infeasible
        if(infeasible == 0){
          infeasible = p;
        }else{
          if(p->pid < infeasible->pid){
            infeasible = p;
          }
        }
      }
    }
  }

  if(missed){
    result.process = missed;
    result.allocated_time = 0;
    return result;
  }
  if(feasible){
    
    int remaining = feasible->exec_time - feasible->current_exec_ticks;
    int deadline_delta = feasible->next_deadline - (int)ticks;
    int allocated = remaining < deadline_delta ? remaining : deadline_delta;

    for(struct proc *p = proc; p < &proc[NPROC]; p++){
      if(!p->is_real_time)  // not real_time
        continue;
       
      if(!p->waiting_for_period)   // not (sleeping and waiting)
        continue;

      int f_remaining = feasible->exec_time - feasible->current_exec_ticks;
      if(p->next_period_start >= (int)ticks + f_remaining)   // never preempt
        continue;
      
      if(p->next_period_start + p->period > feasible->next_deadline)
        continue;

      if(p->next_period_start + p->period == feasible->next_deadline && p->pid > feasible->pid)
        continue;

      int wake_delta = p->next_period_start - (int)ticks;
      if(allocated > wake_delta){
        allocated = wake_delta;
      }

    }
    result.process = feasible;
    result.allocated_time = allocated;
    
    return result;
  }
  if(infeasible){
    result.process = infeasible;
    result.allocated_time = 0;
  }

  return result;
}

struct sched_result
schedule_priority(void)
{
  // TODO, mp3 part 2, priority
  struct sched_result result = {0, 0};
  struct proc *best = 0, *missed = 0;
  for(struct proc *p = proc; p < &proc[NPROC]; p++){
    if(p->state == RUNNABLE && p->is_real_time){
      int remaining = p->exec_time - p->current_exec_ticks;
      
      if((int)ticks >= p->next_deadline && remaining > 0){  // missed
        if(missed == 0 || p->pid < missed->pid){
          missed = p;
        }
        continue;
      }
      
      if(best == 0)
        best = p;

      if(p->priority < best->priority)
        best = p;

      if(p->priority == best->priority && p->pid < best->pid)
        best = p;
    }
  }

  if(missed){
    result.process = missed;
    result.allocated_time = 0;
    return result;
  }
  if(best){
    int remaining = best->exec_time - best->current_exec_ticks;
    int deadline_delta = best->next_deadline - (int)ticks;
    int allocated = remaining < deadline_delta ? remaining : deadline_delta;

    for(struct proc *p = proc; p < &proc[NPROC]; p++){
      if(!p->is_real_time)  // not real_time
        continue;
       
      if(!p->waiting_for_period)   // not (sleeping and waiting)
        continue;

      int b_remaining = best->exec_time - best->current_exec_ticks;
      if(p->next_period_start >= (int)ticks + b_remaining)   // never preempt
        continue;
      
      if(p->priority > best->priority) 
        continue;

      if(p->priority == best->priority && p->pid > best->pid)
        continue;

      int wake_delta = p->next_period_start - (int)ticks;
      if(allocated > wake_delta){
        allocated = wake_delta;
      }

    }

    result.process = best;
    result.allocated_time = allocated;
  }

  return result;
}

void on_cycle_ended(struct proc *p, int deadline_miss)
{
  // TODO, mp3 part 2, DBP
  // Update DBP history and distance with deadline outcome
  int outcome = deadline_miss ? 0 : 1;

  for(int i = p->firm_k - 1; i > 0; i--){
    p->history[i] = p->history[i - 1];
  }
  p->history[0] = outcome;
 
  int success_cnt = 0;
  int idx = -1;

  for(int i = 0; i < p->firm_k; i++){
    if(p->history[i]){
      success_cnt++;
      if(success_cnt == p->firm_m){          
        idx = i;
        break;
      }
    }
  }

  if(idx == -1){
    p->current_distance = 0;
  }else{
    p->current_distance = p->firm_k - idx;
  }

}

struct sched_result
schedule_dbp(void)
{
  // TODO, mp3 part 2, DBP
  struct sched_result result = {0, 0};
  struct proc *best = 0, *missed = 0;

  for(struct proc *p = proc; p < &proc[NPROC]; p++){
    if(p->state == RUNNABLE && p->is_real_time){

      int remaining = p->exec_time - p->current_exec_ticks;

      if((int)ticks >= p->next_deadline && remaining > 0){     // missed
        if(missed == 0){
          missed = p;
        }else{
          if(p->pid < missed->pid){
            missed = p;
          }
        }
        continue;
      }
      
      if(best == 0)
        best = p;
      
      if(p->current_distance < best->current_distance)
        best = p;

      if(p->current_distance == best->current_distance && 
                    p->next_deadline < best->next_deadline)
        best = p;

      if(p->current_distance == best->current_distance && 
               p->next_deadline == best->next_deadline &&
               p->pid < best->pid)
        best = p;
    }
  }

  if(missed){
    result.process = missed;
    result.allocated_time = 0;
    return result;
  }
  if(best){
    int remaining = best->exec_time - best->current_exec_ticks;
    int deadline_delta = best->next_deadline - (int)ticks;
    int allocated = remaining < deadline_delta ? remaining : deadline_delta;

    for(struct proc *p = proc; p < &proc[NPROC]; p++){
      if(!p->is_real_time)  // not real_time
        continue;
       
      if(!p->waiting_for_period)   // not (sleeping and waiting)
        continue;

      int b_remaining = best->exec_time - best->current_exec_ticks;
      if(p->next_period_start >= (int)ticks + b_remaining)   // never preempt
        continue;

      if(p->current_distance > best->current_distance)
        continue;

      if(p->current_distance == best->current_distance && 
                    p->next_deadline > best->next_deadline)
        continue;

      if(p->current_distance == best->current_distance && 
               p->next_deadline == best->next_deadline &&
               p->pid > best->pid)
        continue;

      int wake_delta = p->next_period_start - (int)ticks;
      if(allocated > wake_delta){
        allocated = wake_delta;
      } 

    }

    result.process = best;
    result.allocated_time = allocated;
  }

  return result;
}
