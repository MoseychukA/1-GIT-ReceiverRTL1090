#if defined(ENERGIA_CC1310_LAUNCHXL) || defined(ENERGIA_E70_XXXT14S2)

#ifndef _SMARTRF_SETTINGS_H_
#define _SMARTRF_SETTINGS_H_

//*********************************************************************************
// Generated by SmartRF Studio version 2.11.0 (build#126)
// The applied template is compatible with CC13x0 SDK 2.30.xx.xx
// Device: CC1310 Rev. 2.1 (Rev. B)
//
//*********************************************************************************

#define  DeviceFamily_constructPath(x)  <x>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>

// TI-RTOS RF Mode Object
extern const RF_Mode RF_prop;

// RF Core API commands
extern const rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup;
extern const rfc_CMD_FS_t RF_cmdFs;
extern const rfc_CMD_PROP_TX_t RF_cmdPropTx;
extern const rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv;

// RF Core API Overrides
extern uint32_t pOverrides[];

#endif // _SMARTRF_SETTINGS_H_
#endif // ENERGIA_CC1310_LAUNCHXL
