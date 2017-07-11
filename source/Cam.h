#ifndef CAM_H
#define CAM_H
// ===== Settings =====
// the camera has about 300 rows and 400 cols ,
// but we can not handle that much ,
// so we get 1 row of image for every IMG_STEP rows of camera ,
// and totally get IMG_ROWS rows.

#define IMG_ROWS 64
#define IMG_COLS 128
#define IMG_STEP 2      // TODO: 远多近少

#define CAM_STEP 1 //cam数据处理间距
#define CAM_FAR 10//最远对应cam行数
#define CAM_NEAR 60//最近对应cam行数

#define BLACK_HEIGHT 5
#define BLACK_WIDTH  27

#define PI 3.141593

// ====== Global Variables ======

extern u8 cam_buffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];


// ===== APIs ======

  // write your algorithm in this func
void Cam_Algorithm();

  // Init
void Cam_Init();

//宏定义======================
#define CAM_WID 128//摄像头有效宽度//与摄像头安放位置有关//正常值128
#define Dir_Kp 6.666//0.666    //舵机比例控制参数  8               //6
#define Dir_Kd 26.666  //舵机微分控制参数     //6    12抖得很厉害
#define thr 85//黑白阈值，目前无需调      //70    //新摄像头需要调
#define ROAD_SIZE 50 //利用的摄像头数据行数
#define WEIGHT_SIZE 10 //实际加权并控制舵机的行数
//类型定义=======================================
typedef struct {
  int left;
  int right;
  int mid;
  int width;
}Road;//由近及远存放
//枚举类型================================

enum road_state {straight, curve, roundabout, obstacle, cross}; // 前方道路状态
enum overtake_state {no_overtake, in_overtake};     // 超车状态
enum car_type {leader=1, follower=2}; // 前后车标志
enum car_state {stop, test, normal_drive};   // 智能车状态
enum remote_state {bt_no, bt_prepare, bt_start, bt_finish, bt_forbid, bt_stop, bt_adjust, bt_accelerate};  // 蓝牙通讯
enum roundabout_state {round_no, round_prepare, round_enter, round_in, round_exit}; // 环岛状态
enum roundabout_choice {round_choice_no, round_left, round_right, round_both}; // 环岛内的方向选择
enum cross_state {no_cross, cross_detect, cross_stop, cross_back, cross_go};//十字状态
enum cross_turn {cross_no, cross_left, cross_right, cross_close}; // 十字内的方向选择
enum obstacle_state {obstacle_no, obstacle_pre, obstacle_go};
enum obstacle_type {no_obstacle, obstacle_good, obstacle_bad, obstacle_cross};//障碍种类
enum obstacle_pos {obstacle_right, obstacle_left};//障碍物位置
//函数定义======================================
void Cam_B_Init();//初始化Cam_B
float constrain(float lowerBoundary, float upperBoundary, float input);//控制上下限的函数
int constrainInt(int lowerBoundary, int upperBoundary, int input);//控制上下限的函数(integer专用)
bool is_stop_line(int target_line);//判断是否为终止行/起点行
bool is_hole(int row);   //检测黑洞
bool isWider(int row,int road_width_thr);//检测路宽
//================================================================
//================================================================
//extern 声明：
//特别声明（两车不同的部分）========================
extern float weight[6][10];
extern int MAX_SPEED;
extern int MIN_SPEED;
extern int road_B_near;
extern int road_width_near;
extern int road_B_far;
extern int road_width_far;

//通用·赛道识别================================
extern Road road_B[ROAD_SIZE];//由近及远存放
extern float mid_ave;//road中点加权后的值
extern int valid_row;//与有效行相关，后期用来速控
extern int valid_row_thr;//有效行阈值，区分直道和大弯道
extern enum car_state car_state;//智能车状态标志 0：停止  1：测试舵机  2：正常巡线
extern enum road_state road_state;//前方道路状态 1、直道   2、弯道  3、环岛  4、障碍 5、十字
                  //2 状态下减速//双车======================================
extern enum car_type car_type;//前后车标志 1=前车 2=后车
extern bool flag_stop;
extern enum overtake_state overtake_state;//超车状态      0=无超车     1=主动超车（保持速度或者加速）          2=被超车（减速或停车） 
extern enum remote_state remote_state;//蓝牙通讯   
//0=各自正常行驶   
//1=我（前车）可以被超车，你（后车）收到后设overtake_state为1  
//2=我（后车）开始超车，你（前车）收到后设overtake_state为2      
//3=我（后车)完成超车，我和你（前车）收到后设overtake_state为0，同时两车前后车状态反转
//？？4=我（后车）超不了车，（因为你留的空不够？），咱继续跑……just a joke
//超车时机：启动超车（利用延时）   十字超车    环岛超车    直道超车        //暂时不需要把这个状态通讯？

//环岛检测与处理========================================
extern int check_round_farthest;  //双线延长检测黑洞存在时，最远检测位置，cam_buffer下标，越小越远，不可太小，大概10也就是road_B最远检测点就好
extern int time_cnt;//环岛计时
extern int road_hole_row;//road_B下标 用于检测
extern enum roundabout_state roundabout_state;//0-非环岛 1-预入环岛（直道） 2-入环岛（转向） 3-在环岛 4-出环岛（转向）      注：非零的时候会锁定环岛状态
extern enum roundabout_choice roundabout_choice;//0-未选择 1-左 2-右 3-左右皆可(不用)      //单车能够选择最短路径时要初始化为0
extern int cnt_thr;//检测拐点最远的位置限制，不能太远（即不能太大）否则在环岛检测到的不准，不能太小否则检测不到或者检测到的时候已经晚了
extern int jump_thr;//两个拐点检测的阈值
extern int jump[2][2];//存拐点坐标 0左 1右 0-x 1-y
extern bool flag_left_jump,flag_right_jump;
extern double suml,sumr;//为画图而改成全局变量

//终点识别================
extern u8 is_stopline;
extern u8 cnt_zebra;
extern u8 delay_zebra1, delay_zebra2;

//十字弯处理==============

extern int left3;
extern int right3;
extern int flag_cross; //十字的判断条件
extern int cross_cnt; //十字弯计数
extern int cross_turn; //在十字弯是否靠右停下
extern int cross_times; // 判断成十字的次数
extern int buf_time;    //————————————
extern int right_time;//——————————————
extern int left_time;//——————————————
extern int cross_end; //判断十字是否结束
extern int flag_wide;
extern int wait_time;

//障碍处理==================
extern enum obstacle_type obstacle_type;
extern enum obstacle_state obstacle_state;
extern enum obstacle_pos obstacle_pos;
extern int start_row;
extern int last_row;
extern int obstacle_time_cnt;
//观察·速控========================================================
extern float motor_L;//=MIN_SPEED;
extern float motor_R;//=MIN_SPEED;
extern float max_speed;//=MAX_SPEED;
extern float min_speed;//=MIN_SPEED;

//OLED调参
extern PIDInfo debug_dir;
#endif