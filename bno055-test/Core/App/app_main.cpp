/*
 * app_main.cpp
 *
 *  Created on: Jun 28, 2024
 *      Author: jumpei
 */

#include "app_main.h"

uint8_t aa=10;
uint8_t bb=30;


void app_init();
void app_update();

void app_init(){
	//初期化処理を書く
}


void app_update(){
	//更新処理を書く
}



void app_main(){
	//触らない。
	app_init();
	while(1){
		app_update();
	}
}
