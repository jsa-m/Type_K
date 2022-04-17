/*
 * methodsJS.h
 *
 *  Created on: Jun 25, 2021
 *      Author: joaquin
 */

#ifndef METHODSJS_H_
#define METHODSJS_H_
uint8_t Tamb_to_Kvol(uint32_t temperature);
uint16_t V_to_ktemp(uint32_t ambient_temp, uint32_t V_k);
void ema_filter (uint32_t new_value, uint32_t *St, uint32_t *St_prev);
uint32_t round_temp (uint32_t temp);
uint16_t chartodec(uint8_t d3, uint8_t d2, uint8_t d1);

#endif /* METHODSJS_H_ */
