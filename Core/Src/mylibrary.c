/*
 * mylibrary.c
 *
 *  Created on: Feb 7, 2020
 *      Author: Viktor Vano
 */

#include "mylibrary.h"


char buffer[50];
char ketqua;
char resend_on[] = "LED is ON\r\n";
char resend_off[] = "LED is OFF\r\n";
uint8_t buffer_index = 0;
char bufA[50], bufB[50], bufC[50];
int dataA, dataB, dataC;

char* pop_child_str(char* str1,char* str2){
  const int len = strlen(str2);
  char str3[128] = "";
  char *p1 = str1, *p2;

    /* Tạo vòng lặp để xóa hết chuỗi con */
  while((p2 = strstr(p1,str2)) != NULL) { /*Tìm vị trí chuỗi con bằng hàm strstr*/
    strncat(str3,p1,p2 - p1);   /* Nối các phần không chứa chuỗi con */
    p1 = p2 + len;      /* Dịch chuyển con trỏ sang vị trí tìm kiếm tiếp theo */
  }
  return strcat(str3,p1);
//  printf("%s\n",str3);

}

char* pop_str_last(char* str){
    const int len = strlen(str);
    if( len == 0 ) {
        return '\0';      // Nếu chuỗi ban đầu là Null thì trả về '\0'
    } else {
        /* Dịch chuyển ký tự kết thúc chuỗi để xóa ký tự cuối cùng*/
        str[len-1] = '\0';
        return str;// Trả về chuỗi kết quả
    }
}

uint8_t string_compare(char array1[], char array2[], uint16_t length){
	 uint8_t comVAR=0, i;
	 for(i=0;i<length;i++)
	   	{
	   		  if(array1[i]==array2[i])
	   	  		  comVAR++;
	   	  	  else comVAR=0;
	   	}
	 if (comVAR==length)
		 	return 1;
	 else 	return 0;
}

void Message_handler(){
//	switch (msg.buffer[0]){
//			case 'B':
//				Robot_Move(0.3, 0, 0);
//				break;
//			case 'F':
//				Robot_Move(0.3, 180, 0);
//				break;
//			case 'R':
//				Robot_Move(0.3, 270, 0);
//				break;
//			case 'L':
//				Robot_Move(0.3, 90, 0);
//				break;
//			case 'O':
//				Robot_Move(0, 0, 0.3);
//				break;
//			case '0':
//				Robot_Move(0, 0, 0);
//				break;
//			default:
//				printf("In switch now...");
//				break;
//			}
//	switch (buffer[0]){
//	case FORWARD:
//		Robot_Move(0.2, 225, 0);
//		break;
//	}
////	if(buffer[0]== 'A'){
////		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
////		strcpy(bufA, pop_child_str(pop_str_last(buffer),"A"));
////		dataA = atoi(bufA);
////	}
////	if(buffer[0]== 'B'){
////		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
////		strcpy(bufB, pop_child_str(pop_str_last(buffer),"B"));
////		dataB = atoi(bufB);
////	}
////	if(buffer[0]== 'C'){
////		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
////		strcpy(bufC, pop_child_str(pop_str_last(buffer),"C"));
////		dataC = atoi(bufC);
////	}

	memset(buffer, 0, sizeof(buffer));
	buffer_index = 0;
}
