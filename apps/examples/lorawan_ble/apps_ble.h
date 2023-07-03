/*!
 * @file      app_ble.h
 *
 * @brief     Functions for enabling BLE
 *
 */



#ifndef APPS_BLE_H
#define APPS_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief  Enable Softdevice and BLE libraries
 *
 */
void apps_ble_init(void);


#ifdef __cplusplus
}
#endif

#endif  // APPS_BLE_H