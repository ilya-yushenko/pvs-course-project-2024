/*
 * scheduler.c
 *
 *  Created on: 14 2017.
 *      Author: Maxim
 */
#include "scheduler.h"

__IO static tptr_t  TaskQueue[TaskQueueSize+1];       // Task queue
__IO static ptimer_t MainTimer[MainTimerQueueSize+1]; // Timer queue

// Scheduler initialization
inline void InitScheduler()
  {
    uint8_t index;

    for (index = 0; index != TaskQueueSize + 1; index++)      // Fill task queue with  Idle task
      {
        TaskQueue[index] = Idle;
      }

    for (index = 0; index != MainTimerQueueSize + 1; index++) // Fill timer queue with zeros
      {
        MainTimer[index].GoToTask = Idle;
        MainTimer[index].Time = 0;
      }
  }

// Idle task
inline void  Idle()
{

}

// Function sets the task to queue. The argument is task pointer
void SetTask(tptr_t TS)
  {
    uint8_t index = 0;
    uint8_t interrupts_enable = ~__get_PRIMASK(); // Get the state of interrupts
    // If interrupts are enabled, disable them
    if (interrupts_enable)
      {
        __disable_irq();
      }
    // Go thru task queue. Looking for the empty cell (Idle task)
    while (TaskQueue[index] != Idle)
      {
        index++;
        // If the queue is full then return without result.
        if (index == TaskQueueSize + 1)
          {
            if (interrupts_enable) __enable_irq();    // If interrupts were enabled, enable them back
            return;
          }
      }
    // If there is free cell in task queue set the task in this cell
    TaskQueue[index] = TS;
    if (interrupts_enable) __enable_irq();            // If interrupts were enabled, enable them back
  }

// Function sets the task to timer queue. The arguments are task pointer and time delay (in ms)
void SetTimerTask(tptr_t TS, uint32_t NewTime)
  {
    uint8_t index = 0;
    uint8_t interrupts_enable = ~__get_PRIMASK();     // Get the state of interrupts
    // If interrupts are enabled, disable them
    if (interrupts_enable)
      {
        __disable_irq();
      }
    // Go thru timer queue
    for (index = 0; index != MainTimerQueueSize + 1; ++index)
      {
    		// If there is the same task in the queue
        if (MainTimer[index].GoToTask == TS)
          {
            MainTimer[index].Time = NewTime;          // Rewrite the time delay
            if (interrupts_enable) __enable_irq();    // If interrupts were enabled, enable them back
            return;                                   // Return from function!
          }
      }
    // If there isn't same task in the queue then look for empty cell
    for (index = 0; index != MainTimerQueueSize + 1; ++index)
      {
        if (MainTimer[index].GoToTask == Idle)
          {
            MainTimer[index].GoToTask = TS;             // Set the task pointer
            MainTimer[index].Time = NewTime;            // Set the time delay
            if (interrupts_enable) __enable_irq();      // If interrupts were enabled, enable them back
            return;                                     // Return from function!
          }
      }
  }


// Scheduler (Task manager)
inline void TaskManager()
  {
    uint8_t index = 0;
    tptr_t GoToTask = Idle;   // Initialize the pointer to Idle

    __disable_irq();          // Disable interrupts
    GoToTask = TaskQueue[0];  // Get the first item from the queue

    if (GoToTask == Idle)     // If there is empty cell
      {
        __enable_irq();       // Enable interrupts
        (Idle)();             // Go to default task Idle
      }
    else
      {
    		// In other case shift the queue
        for (index = 0; index != TaskQueueSize; index++)
          {
            TaskQueue[index] = TaskQueue[index + 1];
          }
        // Set the Idle task in the last cell of the queue
        TaskQueue[TaskQueueSize] = Idle;

        __enable_irq();       // Enable interrupts
        (GoToTask)();         // Go to the task
      }
  }

// Timer service
inline void TimerService()
  {
    uint8_t index;
    // Go thru timer queue
    for (index = 0; index != MainTimerQueueSize + 1; index++)
      {
    		// If the cell is empty go to the next iteration
        if (MainTimer[index].GoToTask == Idle) continue;
        // If timer delay is not empty then decrement the timer
        if (MainTimer[index].Time > 1)
          {
            MainTimer[index].Time--;
          }
        else
          {
        		// If the time is zero, set the task to the task queue
            SetTask(MainTimer[index].GoToTask);
            MainTimer[index].GoToTask = Idle;   // Mark cell as empty
          }
      }
  }

