//CAR1简化版-调参版
#include "includes.h"

// ====== Variables ======
// ---- Global ----
u8 cam_buffer_safe[BLACK_WIDTH*2];
u8 cam_buffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];   //64*155，把黑的部分舍去是59*128
//特别声明（两车不同的部分）=================================================
float weight[6][10] ={
{0,0,0,0,0,     0,0,0,0,0},   //0 停车
{1,1,1,1,1,     1,1,1,1,1},// 1 直道
//{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454},
{1.00,1.03,1.14,1.54,2.56,               3.29,4.16,6.00,7.16,5.29},
//{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454},// 2 弯道
{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454},// 3 环岛
{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454},// 4 障碍
{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454} // 5 十字
//其他备选权重：
//{1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29},
//{1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29},
//{1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29}
//{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454}
};//本来是为直、弯、环岛三个路况分别设置的权重，低速下不用考虑，高速可能会有细微区别。
int MAX_SPEED=22;
int MIN_SPEED=14;
int road_B_near=0;//用于观察较近处的路宽判断是否会有分道，road_B下标，越小越近，其值与road_width_thr锁定
                  //CAR1对应路宽 5 -> 直道50+ or 弯道70+ or 入环岛或十字110+ or 出环岛可能80~100+ 
                  //目前CAR1--10  CAR2--0
int road_width_near=83;//该值通过观察check_near对应行的正常路宽来确定
int road_B_far=40;
int road_width_far=40;//未确定
//由以上四个数据可以算出直道上一定距离处看到的路宽。//后两者暂时未使用，无需调试
int valid_row_thr=37;//有效行阈值，区分直道S弯和大弯道         //跟摄像头角度有关

//通用·赛道识别================================
int kp_reduce=0;//直道与长直道弱化
int cnt=0;
Road road_B[ROAD_SIZE];//由近及远存放
float mid_ave;//road中点加权后的值
int valid_row=0;//与有效行相关，后期用来速控，鉴别直弯
enum car_state car_state=0;//智能车状态标志 0：停止  1：测试舵机  2：正常巡线
enum road_state road_state = 1;//前方道路状态 1、直道   2、弯道  3、环岛  4、障碍 5、十字
                  //2 状态下减速
//双车======================================
enum car_type car_type=leader;//前后车标志 1=前车 2=后车
bool flag_stop=0;
enum overtake_state overtake_state=0;//超车状态      0=无超车     1=主动超车（保持速度或者加速）          2=被超车（减速或停车） 
enum remote_state remote_state = 0;//蓝牙通讯   
  //0=各自正常行驶    
  //1=我（前车）可以被超车，你（后车）收到后设overtake_state为1  
  //2=我（后车）开始超车，你（前车）收到后设overtake_state为2      
  //3=我（后车)完成超车，我和你（前车）收到后设overtake_state为0，同时两车前后车状态反转
  //？？4=我（后车）超不了车，（因为你留的空不够？），咱继续跑……just a joke
  //超车时机：启动超车（利用延时）   十字超车    环岛超车    直道超车        //暂时不需要把这个状态通讯？

//环岛检测与处理========================================
int check_round_farthest=20;  //双线延长检测黑洞存在时，最远检测位置，cam_buffer下标，越小越远，不可太小，大概10也就是road_B最远检测点就好
int time_cnt=0;//环岛计时
int road_hole_row=40;//road_B下标 用于检测
enum roundabout_state roundabout_state=0;//0-非环岛 1-预入环岛（直道） 2-入环岛（转向） 3-在环岛 4-出环岛（转向）      注：非零的时候会锁定环岛状态
enum roundabout_choice roundabout_choice=2;//0-未选择 1-左 2-右 3-左右皆可(不用)      //单车能够选择最短路径时要初始化为0
int cnt_thr=40;//检测拐点最远的位置限制，不能太远（即不能太大）否则在环岛检测到的不准，不能太小否则检测不到或者检测到的时候已经晚了
int jump_thr=10;//两个拐点检测的阈值
int jump[2][2];//存拐点坐标 0左 1右 0-x 1-y
bool flag_left_jump=0,flag_right_jump=0;
double suml=0,sumr=0;//为画图而改成全局变量
bool flag_ignore=0;//为避免与stopline混淆而写的局部变量
int ignore_time=0;
//终点识别================
u8 is_stopline = 0;
u8 cnt_zebra = 0;
u8 delay_zebra1 = 0, delay_zebra2 = 0;//1 for the first, should pass; 2 for the second, should stop

//十字弯处理==============
int left3;
int right3;
int left6;
int right6;
int flag_cross=0; //十字的判断条件
int cross_cnt=0; //十字弯计数
int cross_turn=0; //在十字弯是否靠右停下
int cross_times=0; // 判断成十字的次数
int buf_time=-1;
int right_time=-1;
int left_time=-1;
int cross_end=0; //判断十字是否结束
int flag_wide=0;
int flag_thin=0;
int wait_time=-1;

//障碍处理==================
enum obstacle_type obstacle_type=no_obstacle;     //障碍物种类
enum obstacle_pos obstacle_pos=obstacle_right;          //障碍物位置
enum obstacle_state obstacle_state;
int start_row=0; // 障碍开始的行数
int last_row;
int obstacle_time_cnt=0;
int obstacle_valid_row=37;

//观察·速控========================================================
float motor_L;//=MIN_SPEED;
float motor_R;//=MIN_SPEED;
float max_speed;//=MAX_SPEED;
float min_speed;//=MIN_SPEED;

//OLED调参
PIDInfo debug_dir;

// ---- Local ----
u8 cam_row = 0, img_row = 0;

void Cam_B_Init()//初始化Cam_B
{
  int i=0;
  for(i=0;i<ROAD_SIZE;i++)
  {
    road_B[i].left=CAM_WID/2;
    road_B[i].right=CAM_WID/2+2;
    road_B[i].mid=CAM_WID/2+1;
  }
  mid_ave=CAM_WID/2+1;
}

//进化·简化版巡线程序
void Cam_B(){
    cnt++;
    if(cnt>100)      cnt=0;
    //===================变量定义====================
    static int dir;//舵机输出
    static u8 state_set=0;//0=未设置 1=已设置        用来区分优先级

    //自下而上·识别道路============================================================
    for(int j=0;j<ROAD_SIZE;j++)//从下向上扫描
    {
      int i;
      //left
      for (i = road_B[j].mid; i > 0; i--){
        if (cam_buffer[60-CAM_STEP*j][i] < thr)
          break;
        }
     /* if(cnt!=0){
        if(abs(road_B[j].left-i)<10)
            road_B[j].left=i;
      }
      else  road_B[j].left = i;*/
      road_B[j].left = i;
      
      //right
      for (i = road_B[j].mid; i < CAM_WID; i++){
        if (cam_buffer[60-CAM_STEP*j][i] < thr)
          break;
        }
      /*if(cnt!=0){
        if(abs(road_B[j].right-i)<10)
            road_B[j].right=i;
      }
      else  road_B[j].right = i;*/
      road_B[j].right = i;
      //width
      road_B[j].width=road_B[j].right-road_B[j].left;
      //mid
      road_B[j].mid = (road_B[j].left + road_B[j].right)/2;//分别计算并存储共计ROAD_SIZE行(对应cam_buffer第12~60行)的mid
      //store
      if(j<(ROAD_SIZE-1))
        road_B[j+1].mid=road_B[j].mid;//后一行从前一行中点开始扫描
    }

    //分门别类·道路分类===========================
    //优先级：停车线 环岛 十字 弯道 直道
    //但是十字可以锁定
    //判断停车线-----------------------------------------------------//最终版本？
    if(is_stopline == 0 && is_stop_line(45) == 1)
    {
      is_stopline++;
      flag_ignore=1;
      delay_zebra1 = 10;        //10
    }
    else if(is_stopline == 1 && delay_zebra1 == 0 && is_stop_line(45) == 1)
    {
      is_stopline++;
      flag_ignore=1;
      delay_zebra1 = 10;        //100
    }
    else if(is_stopline == 2 && delay_zebra1 == 0 && is_stop_line(50) == 1)
    {
      is_stopline++;
      flag_ignore=1;
      delay_zebra2 = 5;         //5
    }
    else if(is_stopline == 3 && delay_zebra2 == 0){
      is_stopline++;
      flag_ignore=1;
    }
    
  /*
    if(flag_ignore==1 && ignore_time<1000)      //这个时间依赖于车速，而且希望“起点”与环岛不要靠的太近，否则真正的环岛也会被屏蔽掉
      ignore_time++;
    else {
      flag_ignore=0;
      ignore_time=0;
    }
    */
   
    
     // 判断障碍 ————————————————————//未测试
    if(state_set==0){
      int obstacle_set=0;
      for(int i_valid=5;i_valid<(30-5);i_valid++) {
        if ((((road_B[i_valid+5].left-road_B[i_valid].left) >= -5 && (road_B[i_valid+5].left-road_B[i_valid].left) < 10) ||  
             ((road_B[i_valid+6].left-road_B[i_valid].left) >= -5 && (road_B[i_valid+6].left-road_B[i_valid].left) < 10) ||
               ((road_B[i_valid+7].left-road_B[i_valid].left) >= -5 && (road_B[i_valid+7].left-road_B[i_valid].left) < 10))
            && (road_B[i_valid].right-road_B[i_valid+1].right) > 15 && (road_B[i_valid].right-road_B[i_valid+2].right) > 15 && (road_B[i_valid].right-road_B[i_valid+3].right) > 15){
          //road_state=4;
          obstacle_set=1;
          obstacle_pos = obstacle_right;
          start_row = i_valid;
          //break;
        }
        
        if ((((road_B[i_valid].right-road_B[i_valid+5].right) >= -5 && (road_B[i_valid].right-road_B[i_valid+5].right)<10) || 
              ((road_B[i_valid].right-road_B[i_valid+6].right) >= -5 && (road_B[i_valid].right-road_B[i_valid+6].right)<10) || 
               ((road_B[i_valid].right-road_B[i_valid+7].right) >= -5 && (road_B[i_valid].right-road_B[i_valid+7].right)<10))
            && (road_B[i_valid+1].left-road_B[i_valid].left) > 15 && (road_B[i_valid+2].left-road_B[i_valid].left) > 15 && (road_B[i_valid+3].left-road_B[i_valid].left) > 15){
          //road_state=4;
          obstacle_set=1;
          obstacle_pos = obstacle_left;
          start_row = i_valid;
          //break;
        }
        
        if (road_B[i_valid].right > 125 && road_B[i_valid].left<5) obstacle_set=0;
      }
      
   //   if (obstacle_set==1) road_state=4;

      /*
      for(int i_valid=5;i_valid<(30-5);i_valid++) {
        if ((road_B[i_valid+5].left-road_B[i_valid].left) >= -5 && (road_B[i_valid+5].left-road_B[i_valid].left) < 10 && (road_B[i_valid].right-road_B[i_valid+1].right) > 15){
          road_state=4;
          obstacle_pos = obstacle_right;
          start_row = i_valid;
          break;
        }
        
        if ((road_B[i_valid].right-road_B[i_valid+5].right) >= -5 && (road_B[i_valid].right-road_B[i_valid+5].right)<10 && (road_B[i_valid+1].left-road_B[i_valid].left) > 15){
          road_state=4;
          obstacle_pos = obstacle_left;
          start_row = i_valid;
          break;
        }   
      }
      */
      
      last_row=-1;
      float obstacle_diff=0;
      int obstacle_mid = 0;
      int bad_flag = 0;
      //判断完成，进行预处理
      if (road_state==4){
        //obstacle_time_cnt=150;  //预留3s时间处理
        
        for(int i=start_row+6;i<(ROAD_SIZE-1);i++){
          if (last_row==-1){
          if (abs(road_B[i].width-road_B[i+5].width)>10){
            last_row=i;
            break;
          }
          }
        }
        
        obstacle_diff = (road_B[last_row].mid-road_B[start_row].mid)/((float)last_row-(float)start_row);
        obstacle_mid = road_B[start_row].mid;
        
        for (int i = start_row; i <=last_row; i++){
          if (obstacle_pos == obstacle_right){
            if (road_B[i].right<=obstacle_mid){
                obstacle_type=obstacle_bad;
                bad_flag = 1;
            }
          }
          else{
            if (road_B[i].left>=obstacle_mid){
              obstacle_type=obstacle_bad;
              bad_flag = 1;
            }
          }
          obstacle_mid += obstacle_diff;
        }
        
        if (bad_flag==0) obstacle_type=obstacle_good;
        
        //根据Car_type进入不同的Case
          if(car_type==follower){   //后车  （较复杂）
            if(overtake_state==in_overtake){
              if(obstacle_type==obstacle_good || obstacle_type==obstacle_cross){
                //正常避障超车
                obstacle_state=obstacle_go;
              }
              else if(obstacle_type==obstacle_bad){
                //先调整再避障超车        //后期
                obstacle_state=obstacle_pre;
              }
              else{
                flag_stop=1;//不应该出现这个else,临时写来避免出问题
              }
            }
            else if(overtake_state==no_overtake){
              //state_set=0;//属于误判，说明前车太慢，通讯：
              UART_SendChar('f');//……………………………………………………//希望不要出现误判中的误判，不然很尴尬
              obstacle_state=obstacle_go;//碰运气超车或者避障
            }
          }
          else if(car_type==leader){                   //前车
            //避障超过去
            obstacle_state=obstacle_go;
          }
          
          if(obstacle_state==obstacle_go){
            obstacle_time_cnt=150;              //调参…………………………………………………………
          }
          
      }
    }
    //区分环岛与十字的延长线法————————————————————
    if(state_set==0 && flag_ignore==0){     //若没有检测到环岛，且不在十字、终点状态下，则进行拐点（jump）检测，如下：//太精细的计算不适合，所以改成一个简单的
      int cnt=0;
     // int tmpl1=0,tmpl2=0,tmpr1=0,tmpr2=0;
      flag_left_jump=0,flag_right_jump=0;
      for(cnt=0;cnt<cnt_thr;cnt++){     //在road_B[0]~[39]之间检查jump
        if(flag_left_jump==0){
          if((road_B[cnt].left-road_B[cnt+5].left)>jump_thr && road_B[cnt].width!=0){
            flag_left_jump=1;
            suml=road_B[cnt].left-road_B[0].left;
            jump[0][0]=road_B[cnt].left;
            jump[0][1]=60-CAM_STEP*cnt;
          }
        }
        
        if(flag_right_jump==0){
          if((road_B[cnt+5].right-road_B[cnt].right)>jump_thr && road_B[cnt].width!=0){
            flag_right_jump=1;
            sumr=road_B[cnt].right-road_B[0].right;
            jump[1][0]=road_B[cnt].right;
            jump[1][1]=60-CAM_STEP*cnt;
          } 
        }       
       if(flag_left_jump==1&&flag_right_jump==1)//检测到两个拐点       
         break;
      }
      if(flag_left_jump==1&&flag_right_jump==1 && road_B[60-jump[0][1]+1].width>5 && road_B[60-jump[1][1]+1].width>5){//若检测到两个拐点，则判断是环岛还是十字，如下：(同时要求两个拐点不能太远，否则没用)
        //比例：suml 或 sumr / cnt*CAM_STEP
        int cnt_black_row=0;//记录黑行个数
        int left_now,right_now;//存当前行扫描边界
        for(int j=cnt;(60-CAM_STEP*j)>check_round_farthest;j++){//最远检测点check_farthest已调参…………………………………………………………
          left_now=jump[0][0]+suml*(j-cnt)/(cnt*CAM_STEP);
          right_now=jump[1][0]+sumr*(j-cnt)/(cnt*CAM_STEP);
          int cnt_black=0;
          for (int i = left_now; i < right_now; i++){
            if (cam_buffer[60-CAM_STEP*j][i] < thr)
              cnt_black++;
          }
          if(cnt_black>(right_now-left_now)*0.8) cnt_black_row++;//弱化条件试一下
          if(cnt_black_row>=1){         //已弱化
            road_hole_row=60-j;
            if(is_hole(road_hole_row) || is_hole(road_hole_row-3) || is_hole(road_hole_row+3)){
           // if(road_B[road_B_near].width<80){
              road_state=3;                       //完成环岛判断
              roundabout_state=1;
              state_set=1;
              time_cnt=0;
            }
            break;
          }
        }
      }
    }
    /*
    //寻找十字————————————————————————
    int cross_middle;
    float cross_diff = (road_B[8].mid-road_B[3].mid)/5.0;
    int cross_hole=0;
    if(state_set==0 && car_type==leader && flag_left_jump==1 && flag_right_jump==1){
      flag_wide=0;
      flag_thin=0;
      for(int i_valid=5;i_valid<(40-3) && flag_cross==0;i_valid++)     //寻找十字弯
      {
        left3 = (road_B[i_valid].left+road_B[i_valid+1].left+road_B[i_valid+2].left)/3;
        right3 = (road_B[i_valid].right+road_B[i_valid+1].right+road_B[i_valid+2].right)/3;
     
        // else valid_row=ROAD_SIZE-3;
        if ((right3-left3) > 125){
          flag_wide=1;
          cross_middle = road_B[i_valid].mid;
        }
      
        if (flag_wide==1){
          left6 = (road_B[i_valid+3].left+road_B[i_valid+4].left+road_B[i_valid+5].left)/3;
          right6 = (road_B[i_valid+3].right+road_B[i_valid+4].right+road_B[i_valid+5].right)/3;
          if (road_B[i_valid].left >= cross_middle || road_B[i_valid].right <= cross_middle) cross_hole=1;
          if ((left6-left3)>15 && (right3-right6)>15){
            flag_thin=1;
          }
          cross_middle += cross_diff;
        }
      
      }

      if (flag_wide==1 && flag_thin==1 && cross_hole==0){
        //state_set=1;
        road_state=5;
        //cross_turn=3;
        //flag_stop=1;
        //buf_time=1000;
      }
      //找到后：
      
      
    }
    */
    //寻找十字————————————————————————
    
    if(state_set==0 && car_type==leader && flag_left_jump==1 && flag_right_jump==1){
      flag_wide=0;
      flag_thin=0;
      for(int i_valid=5;i_valid<(40-3) && flag_cross==0;i_valid++)     //寻找十字弯
      {
        left3 = (road_B[i_valid].left+road_B[i_valid+1].left+road_B[i_valid+2].left)/3;
        right3 = (road_B[i_valid].right+road_B[i_valid+1].right+road_B[i_valid+2].right)/3;
     
        // else valid_row=ROAD_SIZE-3;
        if ((right3-left3) > 125){
          flag_wide=1;
        }
      
        if (flag_wide==1){
          left6 = (road_B[i_valid+3].left+road_B[i_valid+4].left+road_B[i_valid+5].left)/3;
          right6 = (road_B[i_valid+3].right+road_B[i_valid+4].right+road_B[i_valid+5].right)/3;
          if ((left6-left3)>15 && (right3-right6)>15){
            flag_thin=1;
          }
        }
      
      }

      if (flag_wide==1 && flag_thin==1){
        state_set=1;
        road_state=5;
        //cross_turn=3;
        //flag_stop=1;
      }
      //找到后：
      
      
    }
    
            //寻找十字————————————————————————
  /*  if(state_set==0 && car_type==leader && flag_left_jump==1 && flag_right_jump==1){
      flag_wide=0;
      for(int i_valid=5;i_valid<(40-3) && flag_cross==0;i_valid++)     //寻找十字弯
      {
        left3 = (road_B[i_valid].left+road_B[i_valid+1].left+road_B[i_valid+2].left)/3;
        right3 = (road_B[i_valid].right+road_B[i_valid+1].right+road_B[i_valid+2].right)/3;
     
        // else valid_row=ROAD_SIZE-3;
        if ((right3-left3) > 125){
          flag_wide=1;
        }
      
        if (flag_wide==1){
          if ((road_B[i_valid].right-road_B[i_valid].left) < 25){
            break;
          }
        }
      
        if (flag_wide==1 && i_valid==36){
        //  state_set=1;
          road_state=5;
      //    cross_turn=3;
       //   flag_stop=1;
        //  buf_time=100;
        }
      }
      //找到后：
      
      
    }*/
        //寻找十字————————————————————————
    /*
    if(state_set==0 && car_type==leader && flag_left_jump==1 && flag_right_jump==1){
      flag_wide=0;
      for(int i_valid=0;i_valid<(40-3) && flag_cross==0;i_valid++)     //寻找十字弯
      {
        left3 = (road_B[i_valid].left+road_B[i_valid+1].left+road_B[i_valid+2].left)/3;
        right3 = (road_B[i_valid].right+road_B[i_valid+1].right+road_B[i_valid+2].right)/3;
     
        // else valid_row=ROAD_SIZE-3;
        if ((right3-left3) > 125){
          flag_wide=1;
        }
      
        if (flag_wide==1){
          if ((road_B[i_valid].right-road_B[i_valid].left) < 25){
            break;
          }
        }
      
        if (flag_wide==1 && i_valid==36){
        //  state_set=1;
        //  road_state=5;
        //  cross_turn=3;
       //   flag_stop=1;
        //  buf_time=1000;//
        }
      }
      //找到后：
      
      
    }
    */
    
    //最后，区分直弯-----------------------------
    if(state_set==0){
      static int mid_ave3;
      bool flag_valid_row=0;
      for(int i_valid=0;i_valid<(ROAD_SIZE-3) && flag_valid_row==0;i_valid++)     //寻找有效行
      {
        //方法四：根据cam_buffer中间是否为黑
        if(cam_buffer[60-i_valid*CAM_STEP][CAM_WID/2]<thr){        //如果黑
          flag_valid_row=1;
          valid_row=i_valid;
          break;
        }
      }
      if(flag_valid_row==0) valid_row=ROAD_SIZE-3;
      if(valid_row<valid_row_thr){
        road_state=2;                     //弯道
      }
      else {
        road_state=1;                     //直道 & 可以直线通过的S弯
      }
    }
    
    //=============================根据前方道路类型，进行不同的处理
     switch(road_state)
    {
      case 1:   //直道 or 可以直线通过的S弯
        kp_reduce=0.5;
        max_speed=MAX_SPEED;
        min_speed=MIN_SPEED;
        //选左边界最右值和右边界最左值确定新的中点
        /*
        int left_max=0,right_min=CAM_WID;
        for(int i=0;i<22;i++){
          if(road_B[i].left>left_max) left_max=road_B[i].left;
          if(road_B[i].right<right_min) right_min=road_B[i].right;
        }
        for(int i=1;i<22;i++){
          road_B[i].left=left_max;
          road_B[i].right=right_min;
          road_B[i].mid=(left_max+right_min)/2;
        }
        */
        break;
      case 2:   //大弯道
        kp_reduce=-0.1;
       // max_speed=constrain(MIN_SPEED+1,MAX_SPEED, MAX_SPEED-1);//减多少未定，取决于弯道最高速度
        max_speed=MAX_SPEED;
        min_speed=MIN_SPEED;
        break;
      case 3:   //环岛
        kp_reduce=-0.1;
        min_speed=MIN_SPEED;
        max_speed=min_speed+1;
        switch(roundabout_state)
        {
        case 0://非环岛，用于置零，貌似无用
          roundabout_state=0;//0-非环岛 1-入环岛（有分支） 2-在环岛 3-出环岛（有分支）
          time_cnt=0;
          flag_stop=0;
          state_set=0;
          break;
        case 1://检测到环岛
          if(roundabout_choice==0){
            //暂时用右转代替最短路径（注意：小环岛最短路径影响不大，大环岛能否看到出岛位置则是个问题）
            //拨码控制方案：
            //roundabout_choice=SW1()+1;
            // 最短路径识别方案:
            /*
            */
          }
        //  if(isWider(0,120)){//如果路过于宽，认为出现分叉，开始转弯     新车摄像头可能不一样
          max_speed=min_speed;
          for(int i=1;i<25;i++) road_B[i].mid=CAM_WID/2;
         // if(is_hole(15)){      //可改
          if(road_B[0].width==128){
           // time_cnt++;
           // if(time_cnt>=100)   //提速后延时去掉
            roundabout_state=2;
            time_cnt=0;
          }
          break;  
        case 2://入环岛
         // time_cnt++;

          for(int i=1;i<25;i++){   //利用roundabout_choice给mid加偏移量
            if(roundabout_choice==1) road_B[i].mid = 0;//constrain(0,CAM_WID/2-100,road_B[i].mid-100);
            else if(roundabout_choice==2) road_B[i].mid = CAM_WID;//constrain(CAM_WID/2+100,CAM_WID,road_B[i].mid+100);
          }
          if(road_B[road_B_near].width<128){ //如果路宽恢复正常，认为完成入岛//我猜不如下面的时间控制方式有效
            roundabout_state=3;
            time_cnt=0;
          }
         /* else if(time_cnt>25){
            roundabout_state=3;
          }
          */
        /*
          else if(time_cnt>2000){
            roundabout_state=3;
            time_cnt=0;
          }
          */
          break;
        case 3://在环岛内，看不到出岛，当做弯道行驶
           //停车，发通讯：
          
          if(time_cnt>10&&time_cnt<160)
            flag_stop=1;
          else flag_stop=0;
          
       //   min_speed=10;
        //  max_speed=14;// for test
         
          
          //用来检测什么时候出现分叉//2重条件，增大识别的可能，非最终版
         // time_cnt+=100;
          if(flag_stop==0){
            if(roundabout_choice==1)
              if(road_B[45].left<50){
                roundabout_state=4;
                time_cnt=0;
              }
            else if(roundabout_choice==2)
              if(road_B[45].right>100){
                roundabout_state=4;
                time_cnt=0;
              }
            if(road_B[0].width>85 && road_B[0].width<95        //90
               && road_B[15].width>65 && road_B[15].width<75    //70
                 && road_B[30].width>50 && road_B[30].width<60){        //55
                   roundabout_state=4;
                   time_cnt=0;
                 }
          }
         // if(time_cnt>10000) roundabout_state=0;//约5s  大环岛建议去掉该行
          //以上仅用作紧急处理，最好不要触发该条件
          //如果未检测到，时间又长，说明已经出环岛………………………………如果太短可能会因此而出不了环岛锁定状态……………………………………
            //暂不考虑这种情况，因为大环岛与小环岛用时不同，不可一概而论，（更佳方案是检测纯直道，作为出岛标志）
          break;
        case 4://出环岛，又一次分道
           //发通讯：
              //停车测试
          
          if(time_cnt<250)
            flag_stop=1;
          else flag_stop=0;
          
          
         //time_cnt+=100;
          for(int i=1;i<20;i+=(ROAD_SIZE/10)){   //利用roundabout_choice给mid加偏移量//与forced_turn异曲同工
            if(roundabout_choice==1) road_B[i].mid = CAM_WID/2;//constrain(0,CAM_WID/2,road_B[i].mid-20);
            else if(roundabout_choice==2) road_B[i].mid = CAM_WID/2;//constrain(CAM_WID/2,CAM_WID,road_B[i].mid+20);
          }
          //road_width_thr=70;
          if(time_cnt>10){ //如果路宽回复正常，认为出环岛
            roundabout_state=0;
            state_set=0;
            time_cnt=0;
            flag_stop=0;
          }
         // else if(time_cnt>10000) roundabout_state=0;   //2s 未检测到路宽恢复正常则认为出岛
          break;
        default:break;
        }
        break;
        
      case 4://障碍
        kp_reduce=0;
        max_speed=MAX_SPEED;
        min_speed=MIN_SPEED;
        switch(obstacle_state){
        case obstacle_pre:
          //uart
          UART_SendChar('e');
          //state_set=0;
          obstacle_state=obstacle_go;
          obstacle_time_cnt=150;
          break;
          
        case obstacle_go:
          //time_cnt++;
          int left_max=0;
          int right_min=CAM_WID;
          for(int i=0;i<obstacle_valid_row;i++){
            if(road_B[i].left>left_max) left_max=road_B[i].left;
            if(road_B[i].right<right_min) right_min=road_B[i].right;
          }
          for(int i=0;i<obstacle_valid_row;i++){
            //road_B[i].left=left_max;
            //road_B[i].right=right_min;
            road_B[i].mid=(left_max+right_min)/2;
            //加强偏移
            if(road_B[i].mid>CAM_WID*0.7) road_B[i].mid=(road_B[i].mid+road_B[i].right)/2;                      //调参：0.7 0.3
            else if(road_B[i].mid<CAM_WID*0.3) road_B[i].mid=(road_B[i].mid+road_B[i].left)/2;
          }
          //延时
         // if(time_cnt>500){      //后期改为PIT
          if(obstacle_time_cnt==0){
            state_set=0;
            obstacle_type=no_obstacle;
            obstacle_state=obstacle_no;
          }
          //根据摄像头写剩下的处理代码
          break;
        default:break;
        break;
        }
        break;
        
      case 5://十字
        kp_reduce=0;
        max_speed=MAX_SPEED;
        min_speed=MIN_SPEED;
        if (car_type==leader){
          if(buf_time==-1)
            buf_time=100;//25;        //100
          else if(buf_time>0){
            cross_turn=3;
            flag_stop=1;
          }
          else{
            flag_stop=0;
            state_set=0;
          }
          /*
          else if(right_time==-1)
            right_time=12;      //60
          else if (right_time>0){
            cross_turn = 2;
            flag_stop=0;
          }
          else if(wait_time==-1)
            wait_time=100;     //1000
          else if (wait_time>0){        //后判定条件改为蓝牙通讯
            flag_stop=1;
            cross_turn=3;
          }
          else if(left_time==-1)
            left_time=100;     //1400
          else if (left_time>0){
            flag_stop=0;
            cross_turn=1;
          }
          else if (left_time==0){
            car_type=follower;
           // cross_cnt=0;
            buf_time=right_time=left_time=wait_time=-1;
            cross_turn=0;
            flag_stop=0;
            state_set=0;
          }*/
        }
        break;
      default:break;
    }
    
    //================================对十行mid加权：
    float weight_sum=0;
    int step=3;
    for(int j=0;j<10;j++)
    {
      mid_ave += road_B[j*step].mid * weight[road_state][j];
      weight_sum += weight[road_state][j];
    }
    mid_ave/=weight_sum;
    
    //=================================舵机的PD控制
    static float err;
    static float last_err;
    err = mid_ave  - CAM_WID / 2;
  /*  if(err>0)
      dir = (Dir_Kp+debug_dir.kp) * err*err + (Dir_Kd+debug_dir.kd) * (err-last_err);     //舵机转向  //参数: (7,3)->(8,3.5)-(3.5,3)
    else
      dir = -(Dir_Kp+debug_dir.kp) * err*err + (Dir_Kd+debug_dir.kd) * (err-last_err); 
    */
    dir = (Dir_Kp+debug_dir.kp-kp_reduce) * err + (Dir_Kd+debug_dir.kd) * (err-last_err);
  //  if(dir>0)
   //   dir*=1.2;//修正舵机左右不对称的问题//不可删
    last_err = err;
    
    
    //斑马线：
    if(is_stopline > 0 && (delay_zebra1 > 0 || delay_zebra2 > 0))
      dir = 0;
    //十字：
    if(road_state==5){
      //控向
      if (cross_turn==2){
        dir= 145;
      }
      else if (cross_turn == 3){
        dir=0;
      }
      else if (cross_turn == 1){
        dir = 75;
      }
    }
    
    if(is_stopline > 0 && (delay_zebra1 > 0 || delay_zebra2 > 0))
      dir = 0;
    
    
    //舵机输出与手动停车：
    if(car_state!=0){
      dir=constrainInt(-250,250,dir);   //done! //215 200
      Servo_Output(dir);
    }
    else   
      Servo_Output(0);
    
    
    
    //==============速度控制=================
    //PWM以dir为参考，前期分级控制弯道速度；中期分段线性控速；后期找到合适参数的时候，再进行拟合——PWM关于dir的函数
    min_speed=MIN_SPEED;
    max_speed=MAX_SPEED;
    float range=constrain(0,50,max_speed-min_speed);//速度范围大小 
   /* if(flag_stop==1)
     PWM(0,0,&L,&R);
    
    else if (cross_turn==1){
      PWMne(10, 10, &L, &R);
    }*/
    /*
    else if(car_state==2 ){
      //分段线性控速
      if(abs(dir)<50 ){//&& valid_row>valid_row_thr
        motor_L=motor_R=max_speed;
      }
      else if(abs(dir)<95){
        
        motor_L=motor_R=max_speed-0.33*range*(abs(dir)-50)/45;
        if(dir>0) motor_R=constrain(0,motor_R,motor_R*0.7);//右转
        else motor_L=constrain(0,motor_L,motor_L*0.7);//0.9
      }
      else if(abs(dir)<185){    
        motor_L=motor_R=max_speed-0.33*range-0.33*range*(abs(dir)-95)/90;
        if(dir>0) motor_R=constrain(0,motor_R,motor_R*0.8);//右转
        else motor_L=constrain(0,motor_L,motor_L*0.8);//0/8
      }
      else if(abs(dir)<230){
        motor_L=motor_R=max_speed-0.66*range-0.33*range*(abs(dir)-185)/45;
        if(dir>0) motor_R=constrain(0,motor_R,motor_R*0.9);//右转
        else motor_L=constrain(0,motor_L,motor_L*0.9);//0.7
      }//以上的差速控制参数未确定，调参时以车辆稳定行驶为目标
      else{
        motor_L=motor_R=min_speed;
      }
      if(is_stopline == 4)
        PWM(0, 0, &L, &R);
      else PWM(motor_L, motor_R, &L, &R);               //后轮速度
    }
    */
    if(car_state==2 ){
      //控速
      if(valid_row>45 ){//&& valid_row>valid_row_thr
        motor_L=motor_R=max_speed;
      }
      else if(valid_row>0) {
        motor_L=motor_R=max_speed-range*(45-valid_row)/45;
        double x=dir/250;
        if(x>0) motor_R=constrain(0,motor_R,motor_R*(1-0.3*x-0.2*x*x));//右转     //0.3x+0.2x^2  x=dir/200
        else motor_L=constrain(0,motor_L,motor_L*(1+0.3*x+0.2*x*x));
       // if(dir>0) motor_R=constrain(min_speed,motor_R,motor_R-range*dir/195);//右转
       // else motor_L=constrain(min_speed,motor_L,motor_L-range*(-dir)/210);
        /*if(valid_row > 35){
        motor_L=motor_R=max_speed-0.33*range*(45-valid_row)/10;
        if(dir>0) motor_R=constrain(0,motor_R,motor_R*0.5);//右转
        else motor_L=constrain(0,motor_L,motor_L*0.5);//0.9
      }
      else if(valid_row > 25){
        motor_L=motor_R=max_speed-0.33*range-0.33*range*(35-valid_row)/10;
        if(dir>0) motor_R=constrain(0,motor_R,motor_R*0.75);//右转
        else motor_L=constrain(0,motor_L,motor_L*0.75);//0/8
      }
      else if(valid_row > 15){
        motor_L=motor_R=max_speed-0.66*range-0.33*range*(25-valid_row)/10;
        if(dir>0) motor_R=constrain(0,motor_R,motor_R*0.9);//右转
        else motor_L=constrain(0,motor_L,motor_L*0.9);//0.7
        */
      }//以上的差速控制参数未确定，调参时以车辆稳定行驶为目标
      else{
        motor_L=motor_R=min_speed;
      }
      //差速
      
      
      if(is_stopline == 3)
        PWM(0, 0, &L, &R);
      else PWM(motor_L, motor_R, &L, &R);               //后轮速度
    }
   else
   {
      MotorL_Output(0); 
      MotorR_Output(0);
   }
    
}


float constrain(float lowerBoundary, float upperBoundary, float input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

int constrainInt(int lowerBoundary, int upperBoundary, int input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}


bool is_stop_line(int target_line)//终点识别
{
  cnt_zebra = 0;
  for(int i = 0; i < CAM_WID-5; i++)
  {
    if(cam_buffer[target_line][i] > thr
     &&cam_buffer[target_line][i+1] > thr
     &&cam_buffer[target_line][i+2] < thr
     /*&&cam_buffer[target_line][i+3] < thr*/)
      cnt_zebra++;
  }
  if(cnt_zebra > 5)
    return 1;
  else return 0;
}


bool is_hole(int row)   //检测黑洞
{
  int left=0,right=0;//记录左右跳变次数
    if(cam_buffer[row][CAM_WID/2]<thr)
    {
      //left
      int i=CAM_WID/2-1;
      while(i>0){
        if(left==0 && cam_buffer[row][i]>thr){
          left++;
        }
        else if(left==1 && cam_buffer[row][i]<thr){
          left++;
        }
        else if(left==2) break;
        i--;
      }
      //right
      i=CAM_WID/2+1;
      while(i<CAM_WID){
        if(right==0 && cam_buffer[row][i]>thr){
          right++;
        }
        else if(right==1 && cam_buffer[row][i]<thr){
          right++;
        }
        else if(right==2) break;
        i++;
      }
    }
   // bool static hole=0;
    if(left>=1 && right>=1)     //左右至少各经历一次黑->白的跳变
      return 1;
    else return 0;
        
}

bool isWider(int row,int road_width_thr)
{
  int wid=0;
  for(int i=-1;i<2;i++)
    wid+=(road_B[row+i].right-road_B[row+i].left);
  wid /= 3;
  if(wid>road_width_thr)
    return 1;
  else return 0;
  
}


// 旧cam，不用====== 
void Cam_Algorithm(){
  static u8 img_row_used = 0;
  
  while(img_row_used ==  img_row%IMG_ROWS); // wait for a new row received
  
  // -- Handle the row --  
  
  if (img_row_used >= BLACK_HEIGHT) {     //前5行黑的不要
    for (int col = 0; col < IMG_COLS; col++) {
      u8 tmp = cam_buffer[img_row_used][col];
      if(!SW1()) UART_SendChar(tmp < 0xfe ? tmp : 0xfd);
    }
   if(!SW1()) UART_SendChar(0xfe);//0xfe->纯参数读取溢出
  }
  
  //  -- The row is used --
  img_row_used++;
  if (img_row_used == IMG_ROWS) {    //一帧图像完行归零，控制算法启动，进入AI_Run
    img_row_used = 0;

    if(!SW1()) UART_SendChar(0xff);//0xff->异常结束
  }//以上原来是SW1()
}

// ====== Basic Drivers ======

void PORTC_IRQHandler(){
  if((PORTC->ISFR)&PORT_ISFR_ISF(1 << 8)){  //CS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 8);
    if(img_row < IMG_ROWS && cam_row % IMG_STEP == 0 ){
      DMA0->TCD[0].DADDR = (u32)&cam_buffer[img_row][-BLACK_WIDTH];
      DMA0->ERQ |= DMA_ERQ_ERQ0_MASK; //Enable DMA0
      ADC0->SC1[0] |= ADC_SC1_ADCH(4); //Restart ADC
      DMA0->TCD[0].CSR |= DMA_CSR_START_MASK; //Start
    }
    cam_row++;
  }
  else if(PORTC->ISFR&PORT_ISFR_ISF(1 << 9)){   //VS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 9);
    cam_row = img_row = 0;
  }
}

void DMA0_IRQHandler(){
  DMA0->CINT &= ~DMA_CINT_CINT(7); //Clear DMA0 Interrupt Flag
  
  img_row++; 
}

void Cam_Init(){
  // --- IO ---
  
  PORTC->PCR[8] |= PORT_PCR_MUX(1); //cs
  PORTC->PCR[9] |= PORT_PCR_MUX(1); //vs
  PORTC->PCR[11] |= PORT_PCR_MUX(1);    //oe
  PTC->PDDR &=~(3<<8);
  PTC->PDDR &=~(1<<11);
  PORTC->PCR[8] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(10);	//PULLUP | falling edge
  PORTC->PCR[9] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(9);  // rising edge
  PORTC->PCR[11] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  
  NVIC_EnableIRQ(PORTC_IRQn);
  NVIC_SetPriority(PORTC_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
  
  // --- AD ---
  
  /*
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  //ADC1 Clock Enable
  ADC0->CFG1 |= 0
             //|ADC_CFG1_ADLPC_MASK
             | ADC_CFG1_ADICLK(1)
             | ADC_CFG1_MODE(0);     // 8 bits
             //| ADC_CFG1_ADIV(0);
  ADC0->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_MUXSEL_MASK |  // b
                ADC_CFG2_ADACKEN_MASK; 
  
  ADC0->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
  
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA
  
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; // continuous
  
  //PORTC->PCR[2]|=PORT_PCR_MUX(0);//adc1-4a
  
  ADC0->SC1[0] |= ADC_SC1_ADCH(4);
  */
  
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; //ADC1 Clock Enable
  ADC0->SC1[0] &= ~ADC_SC1_AIEN_MASK; //ADC1A
  ADC0->SC1[0] = 0x00000000; //Clear
  ADC0->SC1[0] |= ADC_SC1_ADCH(4); //ADC1_5->Input, Single Pin, No interrupt
  ADC0->SC1[1] &= ~ADC_SC1_AIEN_MASK; //ADC1B
  ADC0->SC1[1] |= ADC_SC1_ADCH(4); //ADC1_5b
  ADC0->SC2 &= 0x00000000; //Clear all.
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA, SoftWare
  ADC0->SC3 &= (~ADC_SC3_AVGE_MASK&~ADC_SC3_AVGS_MASK); //hardware average disabled
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; //Continuous conversion enable
  ADC0->CFG1|=ADC_CFG1_ADICLK(1)|ADC_CFG1_MODE(0)|ADC_CFG1_ADIV(0);//InputClk, ShortTime, 8bits, Bus
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK; //ADC1  b
  ADC0->CFG2 |= ADC_CFG2_ADACKEN_MASK; //OutputClock
    
  // --- DMA ---
  
  SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK; //DMAMUX Clock Enable
  SIM->SCGC7 |= SIM_SCGC7_DMA_MASK; //DMA Clock Enable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(40); //DMA0->No.40 request, ADC0
  DMA0->TCD[0].SADDR = (uint32_t) & (ADC0->R[0]); //Source Address 0x400B_B010h
  DMA0->TCD[0].SOFF = 0; //Source Fixed
  DMA0->TCD[0].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0); //Source 8 bits, Aim 8 bits
  DMA0->TCD[0].NBYTES_MLNO = DMA_NBYTES_MLNO_NBYTES(1); //one byte each
  DMA0->TCD[0].SLAST = 0; //Last Source fixed
  DMA0->TCD[0].DADDR = (u32)cam_buffer;
  DMA0->TCD[0].DOFF = 1;
  DMA0->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(IMG_COLS+BLACK_WIDTH);
  DMA0->TCD[0].DLAST_SGA = 0;
  DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(IMG_COLS+BLACK_WIDTH);
  DMA0->TCD[0].CSR = 0x00000000; //Clear
  DMA0->TCD[0].CSR |= DMA_CSR_DREQ_MASK; //Auto Clear
  DMA0->TCD[0].CSR |= DMA_CSR_INTMAJOR_MASK; //Enable Major Loop Int
  DMA0->INT |= DMA_INT_INT0_MASK; //Open Interrupt
  //DMA->ERQ&=~DMA_ERQ_ERQ0_MASK;//Clear Disable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK; //Enable
  
  NVIC_EnableIRQ(DMA0_IRQn);
  NVIC_SetPriority(DMA0_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
}