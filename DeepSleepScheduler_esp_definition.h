
// -------------------------------------------------------------------------------------------------
// Definition of ESP, included inside of class Scheduler
// -------------------------------------------------------------------------------------------------
#ifndef ESP32_TASK_WDT_TIMER_NUMBER
#define ESP32_TASK_WDT_TIMER_NUMBER 3
#endif
#ifndef ESP8266_MAX_DELAY_TIME_MS
#define ESP8266_MAX_DELAY_TIME_MS 7000
#endif

#define ESP32
private:
void init();

public:
/**
    Callback executed just before sleep start.
*/
void setBeforeSleepCallback(void (*before_sleep_callback)(SleepMethod, uint64_t));

#ifdef ESP32
// ---------------------------------------------------------------------------------------------
public:

/**
    Do not call this method, it is used by the watchdog interrupt.
*/
static void IRAM_ATTR isrWatchdogExpiredStatic();
private:
hw_timer_t *timer = NULL;
#elif ESP8266
public:
#endif
// ---------------------------------------------------------------------------------------------

private:
void taskWdtEnable(const uint8_t value);
void taskWdtDisable();
inline uint64_t wdtTimeoutToDurationMs(const uint8_t value);
void sleepIfRequired();
inline void sleep(uint64_t durationMs, bool queueEmpty);
inline SleepMode evaluateSleepMode();

// unused here, only used for AVR
bool isWakeupByOtherInterrupt() {
  return false;
}
void wdtEnableInterrupt() {}

void (*before_sleep_callback)(SleepMethod, uint64_t) = nullptr;

