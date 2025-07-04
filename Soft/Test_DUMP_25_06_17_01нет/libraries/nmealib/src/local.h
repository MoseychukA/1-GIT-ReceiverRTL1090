/* local header used by libc/time routines */
#include <_ansi.h>
#include <time.h>

#if !defined(ENERGIA_ARCH_CC13XX) && !defined(ENERGIA_ARCH_CC13X2) && \
    !defined(ARDUINO_ARCH_NRF52)  && !defined(ARDUINO_ARCH_SAMD)
#include <pgmspace.h>
#if defined(ARDUINO_ARCH_ESP32)
#include "esp_idf_version.h"
#endif
#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_RP2040) || \
   (defined(ARDUINO_ARCH_ESP32) && ESP_IDF_VERSION_MAJOR>=4)
#define	_AND		,
#define	_CONST		const
#define	_EXFUN(name, proto)		name proto
#define	_DEFUN(name, arglist, args)	name(args)
#endif
#else
#include <avr/pgmspace.h>
#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_SAMD)
#define	_AND		,
#define	_CONST		const
#define	_EXFUN(name, proto)		name proto
#define	_DEFUN(name, arglist, args)	name(args)
#endif
#endif

#define SECSPERMIN	60L
#define MINSPERHOUR	60L
#define HOURSPERDAY	24L
#define SECSPERHOUR	(SECSPERMIN * MINSPERHOUR)
#define SECSPERDAY	(SECSPERHOUR * HOURSPERDAY)
#define DAYSPERWEEK	7
#define MONSPERYEAR	12

#define YEAR_BASE	1900
#define EPOCH_YEAR      1970
#define EPOCH_WDAY      4
#define EPOCH_YEARS_SINCE_LEAP 2
#define EPOCH_YEARS_SINCE_CENTURY 70
#define EPOCH_YEARS_SINCE_LEAP_CENTURY 370

#define isleap(y) ((((y) % 4) == 0 && ((y) % 100) != 0) || ((y) % 400) == 0)

int         _EXFUN (__tzcalc_limits, (int __year));

extern _CONST int __month_lengths[2][MONSPERYEAR];

/* locks for multi-threading */
#ifdef __SINGLE_THREAD__
#define TZ_LOCK
#define TZ_UNLOCK
#else
#define TZ_LOCK __tz_lock()
#define TZ_UNLOCK __tz_unlock()
#endif

void _EXFUN(__tz_lock,(_VOID));
void _EXFUN(__tz_unlock,(_VOID));

