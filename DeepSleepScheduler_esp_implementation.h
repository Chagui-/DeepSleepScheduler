
#ifdef ESP32
#include <esp_sleep.h>
#include <esp32-hal-timer.h>
#include <soc/rtc.h>
#elif ESP8266
#include <limits.h>
#endif

#ifndef LIBCALL_DEEP_SLEEP_SCHEDULER
// -------------------------------------------------------------------------------------------------
// Implementation (usuallly in CPP file)
// -------------------------------------------------------------------------------------------------
#define ESP8266_MAX_DELAY_TIME_WDT_MS 7500
void Scheduler::init() {}

void Scheduler::setBeforeSleepCallback(void (*before_sleep_callback)(Scheduler::SleepMethod sleep_method, uint64_t sleep_duration)) {
  this->before_sleep_callback = before_sleep_callback;
}

#ifdef ESP32
// -------------------------------------------------------------------------------------------------
extern "C"
{
  #include <esp_clk.h>
}

uint64_t Scheduler::getMillis() const {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec)*1000+tv.tv_usec/1000; // substract epoch from 2020
  // return rtc_time_slowclk_to_us(rtc_time_get(), esp_clk_slowclk_cal_get()) / 1000;
}

void IRAM_ATTR Scheduler::isrWatchdogExpiredStatic() {
#ifdef SUPERVISION_CALLBACK
  if (supervisionCallbackRunnable != NULL) {
    // No need to supervise this call as this interrupt has a time limit.
    // When it expires, the system is restarted.
    supervisionCallbackRunnable->run();
  }
#endif

  ets_printf("Watchdog abort by DeepSleepScheduler\n");
  ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
}

/**
   Interrupt service routine called when the timer expires.
*/
void IRAM_ATTR isrWatchdogExpired() {
  Scheduler::isrWatchdogExpiredStatic();
}

void Scheduler::taskWdtEnable(const uint8_t value) {
  if (value != NO_SUPERVISION) {
    const uint64_t durationMs = wdtTimeoutToDurationMs(value);
    if (timer == NULL) {
      // div 80
      timer = timerBegin(ESP32_TASK_WDT_TIMER_NUMBER, 80, true);
      timerAttachInterrupt(timer, &isrWatchdogExpired, true);
    }
    //set time in us
    timerAlarmWrite(timer, durationMs * 1000, false);
    //enable interrupt
    //only works after taskWdtDisable() if yield() is done before
    yield();
    timerAlarmEnable(timer);
  } else {
    taskWdtDisable();
  }
}

void Scheduler::taskWdtDisable() {
  if (timer != NULL) {
    //disable interrupt
    timerAlarmDisable(timer);
    timerDetachInterrupt(timer);
    timerEnd(timer);
    timer = NULL;
  }
}

void Scheduler::taskWdtReset() {
  //reset timer (feed watchdog)
  if (timer != NULL) {
    timerWrite(timer, 0);
  }
}

#elif ESP8266
// -------------------------------------------------------------------------------------------------
uint64_t Scheduler::getMillis() const {
  // on ESP8266 we do not support sleep, so millis() stays correct.
  return millis();
}

void Scheduler::taskWdtEnable(const uint8_t value) {
  const uint64_t durationMs = wdtTimeoutToDurationMs(value);
  ESP.wdtEnable(durationMs);
}

void Scheduler::taskWdtDisable() {
  ESP.wdtDisable();
}

void Scheduler::taskWdtReset() {
  ESP.wdtFeed();
}
#endif
// -------------------------------------------------------------------------------------------------

inline uint64_t Scheduler::wdtTimeoutToDurationMs(const uint8_t value) {
  uint64_t durationMs;
  switch (value) {
    case TIMEOUT_15Ms: {
        durationMs = 15;
        break;
      }
    case TIMEOUT_30MS: {
        durationMs = 30;
        break;
      }
    case TIMEOUT_60MS: {
        durationMs = 60;
        break;
      }
    case TIMEOUT_120MS: {
        durationMs = 120;
        break;
      }
    case TIMEOUT_250MS: {
        durationMs = 250;
        break;
      }
    case TIMEOUT_500MS: {
        durationMs = 500;
        break;
      }
    case TIMEOUT_1S: {
        durationMs = 1000;
        break;
      }
    case TIMEOUT_2S: {
        durationMs = 2000;
        break;
      }
    case TIMEOUT_4S: {
        durationMs = 4000;
        break;
      }
    case TIMEOUT_8S: {
        durationMs = 8000;
        break;
      }
    default: {
        // should not happen
        durationMs = 15;
      }
  }
  return durationMs;
}

void Scheduler::sleepIfRequired() {
  noInterrupts();
  bool queueEmpty = first == NULL;
  interrupts();
  SleepMode sleepMode = IDLE;
  if (!queueEmpty) {
    sleepMode = evaluateSleepMode();
  } else {
    // nothing in the queue
    if (doesSleep()
#ifdef SLEEP_DELAY
        && millis() >= lastTaskFinishedMillis + SLEEP_DELAY
#endif
       ) {
      sleepMode = SLEEP;
    } else {
      sleepMode = IDLE;
    }
  }
  if (sleepMode != NO_SLEEP) {
#ifdef AWAKE_INDICATION_PIN
    digitalWrite(AWAKE_INDICATION_PIN, LOW);
#endif
    if (sleepMode == SLEEP) {
      taskWdtDisable();
      noInterrupts();
      uint64_t currentSchedulerMillis = getMillis();

      uint64_t firstScheduledUptimeMillis = 0;
      if (first != NULL) {
        firstScheduledUptimeMillis = first->scheduledUptimeMillis;
      }

      uint64_t maxWaitTimeMillis = 0;
      if (firstScheduledUptimeMillis > currentSchedulerMillis) {
        maxWaitTimeMillis = firstScheduledUptimeMillis - currentSchedulerMillis;
      }
      interrupts();

      sleep(maxWaitTimeMillis, queueEmpty);
    } else { // IDLE
      yield();
    }
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
#ifdef AWAKE_INDICATION_PIN
    digitalWrite(AWAKE_INDICATION_PIN, HIGH);
#endif
  }
}

inline Scheduler::SleepMode Scheduler::evaluateSleepMode() {
  noInterrupts();
  uint64_t currentSchedulerMillis = getMillis();

  uint64_t firstScheduledUptimeMillis = 0;
  if (first != NULL) {
    firstScheduledUptimeMillis = first->scheduledUptimeMillis;
  }
  interrupts();

  SleepMode sleepMode = NO_SLEEP;
  uint64_t maxWaitTimeMillis = 0;
  if (firstScheduledUptimeMillis > currentSchedulerMillis) {
    maxWaitTimeMillis = firstScheduledUptimeMillis - currentSchedulerMillis;
  }

  if (maxWaitTimeMillis == 0) {
    sleepMode = NO_SLEEP;
  } else if (!doesSleep() || maxWaitTimeMillis < BUFFER_TIME
#ifdef SLEEP_DELAY
             || millis() < lastTaskFinishedMillis + SLEEP_DELAY
#endif
            ) {
    // use IDLE for values less then BUFFER_TIME
    sleepMode = IDLE;
  } else {
    sleepMode = SLEEP;
  }
  return sleepMode;
}

#ifdef ESP32
// -------------------------------------------------------------------------------------------------
void Scheduler::sleep(uint64_t durationMs, bool queueEmpty) {
  bool timerWakeup;
  if (durationMs > 0) {
    esp_sleep_enable_timer_wakeup(durationMs * 1000L);
    timerWakeup = true;
  } else if (queueEmpty) {
#ifdef ESP_DEEP_SLEEP_FOR_INFINITE_SLEEP
    esp_deep_sleep_start(); // does not return
#endif
    timerWakeup = false;
  } else {
    // should not happen
    esp_sleep_enable_timer_wakeup(1);
    timerWakeup = true;
  }


  // if the sleep time is high enough, use deep sleep, else use light sleep
  if (durationMs > 10000L)
  {
    // call callback if defined
    if (before_sleep_callback) {
      before_sleep_callback(SleepMethod::DEEP_SLEEP, durationMs);
    }
    esp_sleep_enable_timer_wakeup(durationMs * 1000L);
    esp_deep_sleep_start();
  } else {
    // call callback if defined
    if (before_sleep_callback) {
      before_sleep_callback(SleepMethod::LIGHT_SLEEP, durationMs);
    }
    esp_light_sleep_start();
  }

  if (timerWakeup) {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER); // huh?
  }
}
#elif ESP8266
// -------------------------------------------------------------------------------------------------
void Scheduler::sleep(uint64_t durationMs, bool queueEmpty) {
#ifdef ESP_DEEP_SLEEP_FOR_INFINITE_SLEEP
  if (queueEmpty) {
    ESP.deepSleep(0); // does not return
  }
#endif

  if (durationMs > ESP8266_MAX_DELAY_TIME_MS) {
    durationMs = ESP8266_MAX_DELAY_TIME_MS;
  }

  // call callback if defined
  if (before_sleep_callback) {
    before_sleep_callback(SleepMethod::ACTIVE, durationMs);
  }

  delay(durationMs);
  ESP.wdtFeed();
}
#endif
// -------------------------------------------------------------------------------------------------

#endif // #ifndef LIBCALL_DEEP_SLEEP_SCHEDULER

