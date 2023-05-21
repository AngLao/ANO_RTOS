#ifndef _GAME_MAP_H_
#define _GAME_MAP_H_

//UWB 原点基准锚A0 位置偏移量设置
//X=0 , Y=0代表原点基准锚被放置于地图左下角顶点处
#define A0_POS_X 0
#define A0_POS_Y 0

typedef struct {
  unsigned short  x;
  unsigned short  y;
} map_dot_t;


map_dot_t mydot = {.x = 50, .y = 560};
//植保无人机地图二维化数组
//此地图最大长度X=5000mm 最大宽度Y=4000mm

typedef struct {
  //地图起飞点加上28个播洒点
  map_dot_t dot[28 + 1];
} plant_map_t;

//每一小格的长和宽
#define cell_x 515
#define cell_y 500

//-----------------X---------------------
#define plant_map_dot28_x (A0_POS_X+ 1420 )
#define plant_map_dot21_x plant_map_dot28_x

#define plant_map_dot20_x (plant_map_dot28_x + cell_x)
#define plant_map_dot27_x  plant_map_dot20_x

#define plant_map_dot19_x (plant_map_dot20_x + cell_x)
#define plant_map_dot26_x  plant_map_dot19_x

#define plant_map_dot4_x  (plant_map_dot19_x + cell_x)
#define plant_map_dot8_x   plant_map_dot4_x
#define plant_map_dot10_x  plant_map_dot4_x
#define plant_map_dot14_x  plant_map_dot4_x
#define plant_map_dot18_x  plant_map_dot4_x
#define plant_map_dot25_x  plant_map_dot4_x

#define plant_map_dot3_x  (plant_map_dot4_x + cell_x)
#define plant_map_dot7_x   plant_map_dot3_x
#define plant_map_dot9_x   plant_map_dot3_x
#define plant_map_dot13_x  plant_map_dot3_x
#define plant_map_dot17_x  plant_map_dot3_x
#define plant_map_dot24_x  plant_map_dot3_x

#define plant_map_dot2_x  (plant_map_dot3_x + cell_x)
#define plant_map_dot6_x   plant_map_dot2_x
#define plant_map_dot12_x  plant_map_dot2_x
#define plant_map_dot16_x  plant_map_dot2_x
#define plant_map_dot23_x  plant_map_dot2_x

#define plant_map_dot1_x  (plant_map_dot2_x + cell_x )
#define plant_map_dot5_x   plant_map_dot1_x
#define plant_map_dot11_x  plant_map_dot1_x
#define plant_map_dot15_x  plant_map_dot1_x
#define plant_map_dot22_x  plant_map_dot1_x

//-----------------Y---------------------
#define plant_map_dot28_y (A0_POS_Y + 650)
#define plant_map_dot27_y  plant_map_dot28_y
#define plant_map_dot26_y  plant_map_dot28_y
#define plant_map_dot25_y  plant_map_dot28_y
#define plant_map_dot24_y  plant_map_dot28_y
#define plant_map_dot23_y  plant_map_dot28_y
#define plant_map_dot22_y  plant_map_dot28_y

#define plant_map_dot21_y (plant_map_dot28_y + cell_y)
#define plant_map_dot20_y  plant_map_dot21_y
#define plant_map_dot19_y  plant_map_dot21_y
#define plant_map_dot18_y  plant_map_dot21_y
#define plant_map_dot17_y  plant_map_dot21_y
#define plant_map_dot16_y  plant_map_dot21_y
#define plant_map_dot15_y  plant_map_dot21_y

#define plant_map_dot14_y (plant_map_dot21_y + cell_y)
#define plant_map_dot13_y  plant_map_dot14_y
#define plant_map_dot12_y  plant_map_dot14_y
#define plant_map_dot11_y  plant_map_dot14_y

#define plant_map_dot10_y (plant_map_dot14_y + cell_y)
#define plant_map_dot9_y   plant_map_dot10_y

#define plant_map_dot8_y  (plant_map_dot10_y + cell_y)
#define plant_map_dot7_y   plant_map_dot8_y
#define plant_map_dot6_y   plant_map_dot8_y
#define plant_map_dot5_y   plant_map_dot8_y

#define plant_map_dot4_y  (plant_map_dot8_y + cell_y)
#define plant_map_dot3_y   plant_map_dot4_y
#define plant_map_dot2_y   plant_map_dot4_y
#define plant_map_dot1_y   plant_map_dot4_y

extern plant_map_t  plant_map;

plant_map_t  plant_map = {  .dot[0]  = {.x = (A0_POS_X + 640 ), .y = (A0_POS_Y + 3220 )}, //起飞点
                            .dot[1]  = {.x = plant_map_dot1_x, .y = plant_map_dot1_y },
                            .dot[2]  = {.x = plant_map_dot2_x, .y = plant_map_dot2_y },
                            .dot[3]  = {.x = plant_map_dot3_x, .y = plant_map_dot3_y },
                            .dot[4]  = {.x = plant_map_dot4_x, .y = plant_map_dot4_y },
                            .dot[5]  = {.x = plant_map_dot5_x, .y = plant_map_dot5_y },
                            .dot[6]  = {.x = plant_map_dot6_x, .y = plant_map_dot6_y },
                            .dot[7]  = {.x = plant_map_dot7_x, .y = plant_map_dot7_y },
                            .dot[8]  = {.x = plant_map_dot8_x, .y = plant_map_dot8_y },
                            .dot[9]  = {.x = plant_map_dot9_x, .y = plant_map_dot9_y },
                            .dot[10] = {.x = plant_map_dot10_x, .y = plant_map_dot10_y },
                            .dot[11] = {.x = plant_map_dot11_x, .y = plant_map_dot11_y },
                            .dot[12] = {.x = plant_map_dot12_x, .y = plant_map_dot12_y },
                            .dot[13] = {.x = plant_map_dot13_x, .y = plant_map_dot13_y },
                            .dot[14] = {.x = plant_map_dot14_x, .y = plant_map_dot14_y },
                            .dot[15] = {.x = plant_map_dot15_x, .y = plant_map_dot15_y },
                            .dot[16] = {.x = plant_map_dot16_x, .y = plant_map_dot16_y },
                            .dot[17] = {.x = plant_map_dot17_x, .y = plant_map_dot17_y },
                            .dot[18] = {.x = plant_map_dot18_x, .y = plant_map_dot18_y },
                            .dot[19] = {.x = plant_map_dot19_x, .y = plant_map_dot19_y },
                            .dot[20] = {.x = plant_map_dot20_x, .y = plant_map_dot20_y },
                            .dot[21] = {.x = plant_map_dot21_x, .y = plant_map_dot21_y },
                            .dot[22] = {.x = plant_map_dot22_x, .y = plant_map_dot22_y },
                            .dot[23] = {.x = plant_map_dot23_x, .y = plant_map_dot23_y },
                            .dot[24] = {.x = plant_map_dot24_x, .y = plant_map_dot24_y },
                            .dot[25] = {.x = plant_map_dot25_x, .y = plant_map_dot25_y },
                            .dot[26] = {.x = plant_map_dot26_x, .y = plant_map_dot26_y },
                            .dot[27] = {.x = plant_map_dot27_x, .y = plant_map_dot27_y },
                            .dot[28] = {.x = plant_map_dot28_x, .y = plant_map_dot28_y },
                         };
#endif
