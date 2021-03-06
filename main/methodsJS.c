/*
 * methodsJS.c
 *
 * Library that includes a set of functions to be used by other projects
 * including filters, temperature measuring methods, etc
 *
 *  Created on: October, 2021
 *      Author: Joaquín
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

uint16_t typek_C_to_uV[451] = {0, 39, 79, 119, 158, 198, 238, 277, 317, 357, 397, 437, 477, 517, 557, 597, 637, 677, 718, 758, 798, 838,
                               879, 919, 960, 1000, 1041, 1081, 1122, 1163, 1203, 1244, 1285, 1326, 1366, 1407, 1448, 1489, 1530, 1571,
                               1612, 1653, 1694, 1735, 1776, 1817, 1858, 1899, 1941, 1982, 2023, 2064, 2106, 2147, 2188, 2230, 2271, 2312,
                               2354, 2395, 2436, 2478, 2519, 2561, 2602, 2644, 2685, 2727, 2768, 2810, 2851, 2893, 2934, 2976, 3017, 3059,
                               3100, 3142, 3184, 3225, 3267, 3308, 3350, 3391, 3433, 3474, 3516, 3557, 3599, 3640, 3682, 3723, 3765, 3806,
                               3848, 3889, 3931, 3972, 4013, 4054, 4096, 4138, 4179, 4220, 4262, 4303, 4344, 4385, 4427, 4468, 4509, 4550,
                               4591, 4633, 4674, 4715, 4756, 4797, 4838, 4879, 4920, 4961, 5002, 5043, 5084, 5124, 5165, 5206, 5247, 5288,
                               5328, 5369, 5410, 5450, 5491, 5532, 5572, 5613, 5653, 5694, 5735, 5775, 5815, 5856, 5896, 5937, 5977, 6017,
                               6058, 6098, 6138, 6179, 6219, 6259, 6299, 6339, 6380, 6420, 6460, 6500, 6540, 6580, 6620, 6660, 6701, 6741,
                               6781, 6821, 6861, 6901, 6941, 6981, 7021, 7060, 7100, 7140, 7180, 7220, 7260, 7300, 7340, 7380, 7420, 7460,
                               7500, 7540, 7579, 7619, 7659, 7699, 7739, 7779, 7819, 7859, 7899, 7939, 7979, 8019, 8058, 8099, 8138, 8178,
                               8218, 8258, 8298, 8338, 8378, 8418, 8458, 8499, 8539, 8579, 8619, 8659, 8699, 8739, 8779, 8819, 8860, 8900,
                               8940, 8980, 9020, 9061, 9101, 9141, 9181, 9222, 9262, 9302, 9343, 9383, 9423, 9464, 9504, 9545, 9585, 9626,
                               9666, 9707, 9747, 9788, 9828, 9869, 9909, 9950, 9991, 10031, 10072, 10113, 10153, 10194, 10235, 10276, 10316,
                               10357, 10398, 10439, 10480, 10520, 10561, 10602, 10643, 10684, 10725, 10766, 10807, 10848, 10889, 10930, 10971,
                               11012, 11053, 11094, 11135, 11176, 11217, 11259, 11300, 11341, 11382, 11423, 11465, 11506, 11547, 11588, 11630,
                               11671, 11712, 11753, 11795, 11836, 11877, 11919, 11960, 12001, 12043, 12084, 12126, 12167, 12209, 12250, 12291,
                               12333, 12374, 12416, 12457, 12499, 12540, 12582, 12624, 12665, 12707, 12748, 12790, 12831, 12873, 12915, 12956,
                               12998, 13040, 13081, 13123, 13165, 13206, 13248, 13290, 13331, 13373, 13415, 13457, 13498, 13540, 13582, 13624,
                               13665, 13707, 13749, 13791, 13833, 13874, 13916, 13958, 14000, 14042, 14084, 14126, 14167, 14209, 14251, 14293,
                               14335, 14377, 14419, 14461, 14503, 14545, 14587, 14629, 14671, 14713, 14755, 14797, 14839, 14881, 14923, 14965,
                               15007, 15049, 15091, 15133, 15175, 15217, 15259, 15301, 15343, 15385, 15427, 15469, 15511, 15554, 15596, 15638,
                               15680, 15722, 15764, 15806, 15849, 15891, 15933, 15975, 16017, 16059, 16102, 16143, 16186, 16228, 16270, 16312,
                               16355, 16397, 16439, 16482, 16524, 16566, 16608, 16651, 16693, 16735, 16778, 16820, 16862, 16904, 16947, 16989,
                               17031, 17074, 17116, 17158, 17201, 17243, 17285, 17328, 17370, 17413, 17455, 17497, 17540, 17582, 17624, 17667,
                               17709, 17752, 17794, 17837, 17879, 17921, 17964, 18006, 18049, 18091, 18134, 18176, 18218, 18261, 18303, 18346,
                               18388, 18431, 18473, 18516 };


//Function that converts the temperature obtained with an external temperature sensor to their
//equivalent voltage (in micro-volts) associated with the k thermocouple
uint16_t Tamb_to_Kvol(uint32_t temperature){
    if(temperature <= 450){
        return typek_C_to_uV[temperature];
    }
    else{
        return 0;
    }
}

//Function that obtains the type k thermocouple temperature applying software compensation. First the ambient temperature
//is converted to its thermocouple voltage, then both voltages (type k, ambient) are added. The temperature is obtained
//looking at the index of the closest voltage value in the look up table.
uint16_t V_to_ktemp(uint32_t ambient_temp, uint32_t V_k){
    uint32_t v_temp = 0, index = 0, value = 0, value_1 = 0;
    v_temp = Tamb_to_Kvol(ambient_temp) + V_k;
    while( (v_temp > typek_C_to_uV[index]) && (index < 450)){
        index +=1;
    }
    value = typek_C_to_uV[index];
    if(index !=0){
        value_1 = typek_C_to_uV[index - 1];
    }
    if(((value-v_temp) < (v_temp - value_1)) || (index == 0)){
        return index;
    }
    else{
        return index-1;
    }

}

//Function that implements a digital low pass filter, applying the exponential moving average function
void ema_filter (uint32_t new_value, uint32_t *St, uint32_t *St_prev){
 *St_prev = *St;
 *St = (1*new_value + 9 **St_prev)/10;
}

//Function to round the temperature. f.e 241 -> 240, 243 -> 245 -> 248 -> 250
uint32_t round_temp (uint32_t temp){
    if(temp % 10 < 3){
        return (temp - temp%10);
    }
    else if ((temp % 10 >= 3) & (temp % 10 < 8)){
        return (temp - temp%10 + 5);
    }
    else{
        return (temp - temp%10 + 10);
    }
}

//Converts number of 3 digits in char format to int, used to translate data coming from BLE
uint16_t chartodec(uint8_t d3, uint8_t d2, uint8_t d1){
	uint16_t dec;
	if(d3 == 0 && d2 == 0 && d1 == 0){
		return dec = 0;
	}
	else if(d3 == 0 && d2 == 0 && d1 != 0){
		return dec = d1 - 48;
	}
	else if(d3 == 0 && d2 != 0 && d1 == 0){
		return dec = d2 - 48;
	}
	else if(d3 == 0 && d2 != 0 && d1 != 0){
		return dec = (d2 - 48)*10 + (d1-48);
	}
	else if(d3 != 0 && d2 == 0 && d1 == 0){
		return dec = (d3 - 48);
	}
	else if(d3 != 0 && d2 == 0 && d1 != 0){
		return dec = (d3 - 48)*10 + (d1-48);
	}
	else if(d3 != 0 && d2 != 0 && d1 == 0){
		return dec = (d3 - 48)*10 + (d2-48);
	}
	else{
		return dec = (d3 - 48)*100 + (d2-48)*10 + (d1-48);
	}
}

