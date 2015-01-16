/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <time.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"



struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

void printLog();

componentservertype lmssrv,camsrv;

 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

#define WHEEL_DIAMETER   0.064/* m *///0.067
#define WHEEL_SEPARATION  0.2686//0.2699//0.2594 //0.2771//0.2873//	/* m */0.252, 0.2629 0.2528 0.2771
#define E_d 1.0074 //1.0057//1.0139 //1.0057 1.01 
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902
#define LOGLENGTH 4000
#define LOGHEIGHT 14
#define LINE_SENS_LENGTH 8
#define IR_SENS_LENGTH 6

/*****************************************
* odometry
*/

typedef struct{ //input signals
		int left_enc,right_enc, enc_r, enc_l; // encoderticks

		// parameters
		double w;	      // wheel separation
		double cr,cl;   // meters per encodertick
	  //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
    double ran_dist;
		double old_x, old_y, x, y;
		double theta, old_theta;
		double ticks;
		} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);
/********************************************
* Sensor Control
*/
typedef struct{
  double linesensor[LINE_SENS_LENGTH];
  int min_line_sensor;
  int max_line_sensor;
  int black_line_follow;
  int lost; //true or false
  int cross; //true or false
  int w_cross; //true or false
  int fork; //true or false
  int fork_test;
  
  double prev_ir_r, prev_ir_l;
  double ir_min;
  double irsensor[IR_SENS_LENGTH];
  double ir_dist[IR_SENS_LENGTH];
  
  double laser[10] ;
  int min_las;
  int min_las_2;
  int min_las_3;
  int min_las_r;
  int min_las_l;
 } sensetype;
 
 void update_sensors(sensetype *s,odotype *q);
 const double linesensor_interp[2][8]={{0.0312,0.0287,0.0326,0.0266,0.0257,0.0270,0.0272,0.0322},
                              {-1.6748,-1.5475,-1.7256,-1.4311, -1.3930,-1.4552,-1.4883,-1.7941}};
 

/********************************************
* Motion control
*/

typedef struct{//input
    int cmd;
		int curcmd;
		//fwd spped
		double speedcmd;//current speed [-1,1]
	  int speed;//[currenty spped[-128,127]
		int t_speed;//target speed[-128,127]
		
		//turn spped
		double speedcmd_t;//current speed [-1,1]
	  int speed_t;//[currenty spped[-128,127]
		int t_speed_t;//target speed[-128,127]
		
		double dist;//aim distance
		double angle;// aim angle
		double delta_v; //controller add up. depends on what calls it(follow line, move straight...)
		double real_dist;
		double left_pos,right_pos;
	
		// parameters
		double w;//wheel base width
		double kfr, kfl, kf; //K meter per encoder adjusment
		double kw, ki,k_follow,k_wall; //proportional controller constants
		double kspeed1,kspeed2,kspeed3;
		//output
		double motorspeed_l,motorspeed_r; 
		int finished;
		// internal variables
		double startpos; //currently unused
		double start_theta;
	  double start_x;
	  double start_y;
	  
	  int flag_collision;
	  int flag_box;
	       }motiontype;

typedef struct{
  
  double cell, xsize, ysize, R_safe,laser_max, theta;
  int map[25][25];//[map_xsize/cell][map_ysize/cell]
  int x,y;//current
  int xg,yg;//goal
  int xcg, ycg;//closens accesible point to the goal. 
  double alpha;
              }maptype;

typedef struct{
    int state,oldstate;
		int time;
		int sub_time;
		int sub_m_flag;
		int sub_w_flag;
		int fl;
		    }smtype;

typedef struct{
		double log[LOGHEIGHT][LOGLENGTH];
		int counter;
		}logdata;
	       
enum {mot_stop=1,mot_move,mot_turn, mot_follow_line};

void update_motcon(motiontype *p, odotype *q, sensetype *s);	       

int fwd(double dist, double speed,int time, int condition);
int follow_line(double speed, char dir,int flag_c,int time, int condition);
int follow_white_line(double speed, double time);
int follow_wall(double speed, char dir, double dist_to_wall ,int time, int condition);
int recover_line(int time);
int turn(double angle, double speed,int time);
int gate_loose(double speed, int time);
int gate_wall(double speed, char dir, double dist_to_wall ,int time);
int push_the_box(double speed, int time);

      
void myspeed(int aim, motiontype *p, odotype *q, sensetype *s);
void myspeed_t(int aim, motiontype *p, odotype *q, sensetype *s);
void sm_update(smtype *p);
void update_map(odotype *q, sensetype *s ,maptype *m);
// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl,*tick;

odotype odo;
smtype mission;
motiontype mot;
sensetype sens;
maptype map;
logdata dataMatrix;

const double BLACK_THRESHOLD = 0.3;
const double WHITE_THRESHOLD = 0.75;
const double robot_length = 0.25;//0.28;
const double gate_width = 0.47;

int cross_counter;
int w_cross_counter;


enum {ms_init, ms_follow_white_line,ms_fwd,ms_follow_line,ms_turn,ms_end, ms_gate_wall, ms_gate_loose, ms_push_the_box};

int main()
{
  
  int running,n=0,arg;
  double dist=0,angle=0;
  //double waypoints [5][2]={{2, 0},{2,2},{0, 2},{0,0.15},{0,0}};
  //int waypointcounter=0;
  int var1, var2;
  cross_counter = 0;
  w_cross_counter = 0;
  sens.cross = 0;
  sens.w_cross = 0;
  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	       exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	       exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	       exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
      tick = getoutputref("tick",inputtable);
     
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
      
     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     len=sprintf(buf,"scanpush cmd='zoneobst'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

}   
   
 
  /* Read sensors and zero our position.
   */
  dataMatrix.counter = 0;
  
  rhdSync();
  odo.enc_r=0;
  odo.enc_l=0;
  odo.w=WHEEL_SEPARATION;
  odo.cl=DELTA_M;
  odo.cr=odo.cl*E_d;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  odo.x = odo.y = odo.old_x = odo.old_y = 0;
  odo.theta = odo.old_theta = 0;
  odo.ran_dist=0;
  odo.ticks = tick->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
  mot.t_speed=0;
  mot.t_speed_t=0;
  mot.kfr=mot.kfl=mot.kf=0.23;
  mot.kw=0.2;
  mot.k_follow=-0.15;//-0.11
  mot.k_wall = 6;
  mot.kspeed1=5;
  mot.kspeed2=5;
  mot.kspeed3=8;
  mot.ki=0.1;
  mot.flag_box = 0;
  running=1; 
  mission.state=ms_init;
  mission.oldstate=-1;
  mission.sub_m_flag=0;
  mission.sub_w_flag=0;
  sens.black_line_follow = 1;
  int speed =0;
  
  map.cell=0.2; map.xsize=5.0; map.ysize=5.0; map.R_safe=0.4; map.laser_max=3.5, map.theta=0.0;
  for (var1=1; var1<25;var1++){
    for (var2=1; var2<25;var2++){
       map.map[var1][var2]=-1;
    }
  }
  map.x=12; map.y=12;
  map.xg=20; map.yg=20;//goal
  map.xcg=map.x, map.ycg=map.y;//closens accesible point to the goal. 
  map.alpha=M_PI/9;
  
  
  
  
  while (running){ 
    if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
     }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }
       

  rhdSync();
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  odo.ticks = tick->data[0];
  update_sensors(&sens, &odo);
  update_odo(&odo);
  update_map(&odo, &sens, &map);
  dataMatrix.counter++;
/****************************************
/ mission statemachine   
*/
   sm_update(&mission);
   switch (mission.state) {
    case ms_init:
       n=4; dist=0.25;angle=90.0;
       mission.state=ms_fwd; //ms_gate_wall; //ms_follow_white_line; //ms_push_the_box; //ms_fwd; //ms_follow_line; //ms_gate_loose; //ms_end; //ms_fwd; //ms_turn; // ms_fwd; //ms_goto_setup; // 
       speed=40;
       mot.flag_collision = 1;
       //waypointcounter=0;
    break;    
    case ms_fwd:
       if (fwd(dist,speed,mission.time, 0)){  
        printf("I finished fwd \n");
        mission.state=ms_gate_wall;
       }
	     //mission.state=ms_end;
    break;
    case ms_turn:
      if (turn(angle,speed/2,mission.time)){
          printf("I finished turn n=%d \n", n);
         n=n-1;
      if (n==0) 
	      mission.state=ms_end;
	    else
	      mission.state=ms_fwd;
	    }
    break;  
    case ms_follow_line:
       //if(sets->lost)
       // mission.state=ms_recoverline;
       if (follow_line(speed, 'l',1,mission.time, sens.cross))  
        mission.state=ms_gate_loose;
    break;
    case ms_gate_wall:
       if (gate_wall(20, 'l', 0.3,mission.time))
       //if (follow_wall(20, 'l', 0.3, mission.time, 0))  
        mission.state=ms_end;
    break;
    case ms_gate_loose:
       if (gate_loose(30, mission.time))  
        mission.state=ms_end;
    break;
    case ms_push_the_box:
      if (push_the_box(speed, mission.time))  
        mission.state=ms_end;
    break;
    case ms_follow_white_line:
      if (follow_white_line(speed, mission.time))  
        mission.state=ms_end;
    break;
    case ms_end:
      mot.cmd=mot_stop;
      running=0;
    break;
   }  
   
  // printf("cr: %f; cl: %f \n", odo.cr, odo.cl);
/*  end of mission  */
  //calc_time(&start);
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot, &odo, &sens);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  //printf(" laser %f \n",laserpar[3]);
  //if (time  % 100 ==0)
    //    printf(" laser %f \n",laserpar[3]);
  //time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;
}/* end of main control loop */
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  printLog(); //Print log to file
  exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
}

void update_odo(odotype *p)
{
  int delta;
  
  
  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  p->enc_r+=delta;
  double delta_rpos = delta * p->cr;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  p->enc_l+=delta;
  double delta_lpos = delta * p->cl;
  
  double Ui = (delta_lpos + delta_rpos) / 2;
  p->theta = p->old_theta + (delta_rpos - delta_lpos)/p->w;
  p->x = p->old_x + Ui*cos(p->theta);
  p->y = p->old_y + Ui*sin(p->theta);
  p->ran_dist+=Ui;
  
  //printf("v=%f \n",(p->x-p->old_x)/0.01);
  
  p->old_x = p->x;
  p->old_y = p->y;
  p->old_theta = p->theta;
  if (dataMatrix.counter<LOGLENGTH)
  {  
    dataMatrix.log[0][dataMatrix.counter] = p->x;
    dataMatrix.log[1][dataMatrix.counter] = p->y;
    dataMatrix.log[2][dataMatrix.counter] = p->theta;
    
  }
  
  //printf("X: %f, Y:%f, TH: %f \n", p->x, p->y, p->theta);
}


void update_motcon(motiontype *p, odotype *q, sensetype *s){ 

if (p->cmd !=0){
     
     p->finished=0;
     switch (p->cmd){
       case mot_stop:
        p->curcmd=mot_stop;
        break;
       case mot_move:
        //uselesd p->startpos=sqrt(pow((q->x)-(p->start_x),2)+pow((q->y)-(p->start_y),2));// (q->x)*(q->x)+(q->y)*(q->y)); //(p->left_pos+p->right_pos)/2;
        p->start_x = q->x;
	      p->start_y = q->y;
        p->start_theta=q->theta;
        
        p->curcmd=mot_move;
       break;
       case mot_turn:
         p->start_theta=q->theta;
	       p->start_x = q->x;
	       p->start_y = q->y;
         p->curcmd=mot_turn;
       break;
       
       
     }
     
     p->cmd=0;
   }
   
   switch (p->curcmd){
     case mot_stop:
       myspeed(p->t_speed, p, q, s);
       p->motorspeed_l=p->speedcmd;//0;
       p->motorspeed_r=p->speedcmd;//0;
       p->finished=1;
     break;
     case mot_move:
       
       if (sqrt(pow((odo.x)-(mot.start_x),2)+pow((odo.y)-(mot.start_y),2))>p->dist){//(p->real_dist > p->dist){ when move is equal for fwd and follo wline
       
          p->finished=1;
	        p->motorspeed_l=0;
          p->motorspeed_r=0;
       }	  
       else {
          
          myspeed(p->t_speed, p, q, s);
                    
          if (!(p->speedcmd+p->delta_v>1||p->speedcmd+p->delta_v<-1||p->speedcmd-p->delta_v>1||p->speedcmd-p->delta_v<-1)) 
          {
	          p->motorspeed_l=p->speedcmd+p->delta_v;//*kfl/kf
            p->motorspeed_r=p->speedcmd-p->delta_v;
          }
          else
          {
            printf("something went wrong, speed out of range");
            p->cmd=mot_stop;
          }
            
       }
       
     break;
     
     case mot_turn:
       myspeed_t(p->t_speed_t, p, q, s);
       if (p->angle>0){
          
	      if (q->theta - p->start_theta < p->angle){
	          p->motorspeed_r=p->speedcmd_t;
	          p->motorspeed_l=-(p->speedcmd_t);
	      }
	      else {	     
          p->motorspeed_r=0;
          p->motorspeed_l=0;
          p->finished=1;
	      }
	    }
	    else {
               
	      if (q->theta-p->start_theta > p->angle){
	          p->motorspeed_l=p->speedcmd_t;
	          p->motorspeed_r=-(p->speedcmd_t);
	      }
	      else {	  
	        p->motorspeed_r=0;   
          p->motorspeed_l=0;
          p->finished=1;
	      }
	    }
     break;
   }   
}   

int fwd(double dist, double speed,int time, int condition){
     //double distance=fabs(tan(mot.start_theta)*(odo.x)-(odo.y)+tan(mot.start_theta)*mot.start_x+mot.start_y)/sqrt(tan(odo.theta)*tan(odo.theta)+1);
     //I controlle rnot implemented
     mot.delta_v=0;//(mot.kw*mot.kspeed1*(odo.theta-mot.start_theta));//+mot.ki*mot.kspeed2*distance)*mot.speedcmd;
     //mot.real_dist=odo.ran_dist;//sqrt(pow((odo.x)-(mot.start_x),2)+pow((odo.y)-(mot.start_y),2));
     if(condition)
     {
        mot.cmd=mot_stop;
        return mot.finished;
     }
   if (time==0){ 
     printf("I'm in fwd!!\n");
     mot.cmd=mot_move;
     mot.t_speed=speed;
     mot.dist=dist;
     odo.ran_dist=0;
     return 0;
   }
   else
     return mot.finished;
   
     
}

int follow_line(double speed, char dir, int flag_c,int time, int condition){
  int i = 0;
  int min = 0;
  int flag = 1;
  if(condition)
 {
    mot.cmd=mot_stop;
    return mot.finished;
 }
  if(sens.fork){
    if(dir == 'r'){
      for(i = 0; i < LINE_SENS_LENGTH; i++){
        if(sens.linesensor[i] < BLACK_THRESHOLD){
          flag = 0;
          if(sens.linesensor[i] < sens.linesensor[min]) min = i;
        }
        //Break after first black line
        if(sens.linesensor[i] > BLACK_THRESHOLD && !flag) break;
      }
    }
    else if(dir == 'l'){
      min = 7;
      for(i = LINE_SENS_LENGTH-1; i >= 0; i--){
        if(sens.linesensor[i] < BLACK_THRESHOLD){
          flag = 0;
          if(sens.linesensor[i] < sens.linesensor[min]) min = i;
        }
        //Break after sfirst black line from right
        if(sens.linesensor[i] > BLACK_THRESHOLD && !flag) break;
      }
    }
    else if(dir == 'c'){
      min = 3.5; //if error this is it
    }
    else{
      min = sens.min_line_sensor;
    }
      sens.min_line_sensor = min;
      if(mot.flag_box && odo.y<-0.4)
        printf("the box is %f m away\n",sens.laser[4]+0.06);
        
  }
   mot.delta_v= mot.k_follow*(!flag_c*sens.min_line_sensor+flag_c*sens.max_line_sensor-3.5)*mot.speedcmd;//follow black or white
   //mot.delta_v= mot.k_follow*(sens.max_line_sensor-3.5)*mot.speedcmd;//follow white.
   //mot.real_dist=odo.ran_dist;
   /*if (sens.lost){ 
      
      recover_line(time-mission.sub_time);
   }*/ 
   if (time==0){ 
     mot.cmd=mot_move;
     mot.t_speed=speed; 
     mot.dist=3;
     odo.ran_dist=0;
     return 0;
   }
   else if(mot.finished == 1)
     return mot.finished;
   else
    return 0;
}
int gate_wall(double speed, char dir, double dist_to_wall ,int time){
  double dist = 0;
  switch(mission.sub_m_flag)
  { 
    case 0:
      mission.fl=1;
      if(dist_to_wall < robot_length+0.15) dist_to_wall = robot_length+0.15; //safety at least 10 cm
      dist=sens.laser[4]-dist_to_wall+robot_length;
      if(fwd(dist, speed, mission.sub_time, 0)){
          mission.sub_m_flag=1;
          mission.sub_time=(-1);
      }
    
    break;
    case 1:
      if(turn(-90, speed/2, mission.sub_time)){
        mission.sub_m_flag=2;
        mission.sub_time=(-1);
      }
    break;
    case 2:
     printf("following wall\n");
     if(follow_wall(speed,  dir,  dist_to_wall ,mission.sub_time, 0)){
        printf("done following wall\n");
        mission.fl=1;
        mission.sub_m_flag= 3;
        mission.sub_time=(-1);
     }
    break;
    case 3:
      if(fwd(tan(M_PI/9) * dist_to_wall + gate_width/2 + robot_length, speed, mission.sub_time, 0)){ // 0.45
      //if line break return 1;
        mission.sub_m_flag=4;
        mission.sub_time=(-1);
      }
    break;
    case 4:
      if(turn(90, speed/2, mission.sub_time)){
        mission.sub_m_flag=5;
        mission.sub_time=(-1);
      }
    break;
    case 5:
      if(fwd(dist_to_wall*2 + 0.10, speed, mission.sub_time, 0)){
        
        mission.sub_m_flag= 6;
        mission.sub_time= (-1);
      }
    break;
    
    case 6:
      if(turn(90, speed/2, mission.sub_time)){
        mission.sub_m_flag=7;
        mission.sub_time=(-1);
      }
    break;
    case 7:
      if(fwd(tan(M_PI/9) * dist_to_wall + gate_width/2 - robot_length, speed, mission.sub_time, 0)){
      //if line break return 1;
        mission.sub_m_flag=2;
        mission.sub_time=(-1);
      }
    break;
    }
    return 0;
}
int follow_wall(double speed, char dir, double dist_to_wall ,int time, int condition){
  double laser_wall=0;
  double dist = 0;
  
  printf("l0: %f, l8: %f\n", sens.laser[0], sens.laser[8]);
  if(dir=='r'){
    if (sens.laser[1]>1.0 && sens.laser[0] > 1.0){
      printf("no wall to follow");
      return 1;
    }
    else{
    laser_wall=sens.laser[0];
    }
  }
  else if(dir=='l'){
    if (sens.laser[7]>1.0 && sens.laser[8] > 1.0){
      printf("no wall to follow");
      return 1;
    }
    else{
      laser_wall=sens.laser[8];
    }
    
  }
    dist = laser_wall-dist_to_wall;
    if(dist > 0.15) dist = 0.15;
    if(dist < -0.15) dist = -0.15;
    
      mot.delta_v=(dist)*-4*mot.speedcmd; //mot.k_wall
      //printf("ir4: %f, delta_v: %f \n", sens.ir_dist[4], mot.delta_v);
  
  if(condition)
  {
     mot.cmd=mot_stop;
     return 1;
  }
  if (time==0){ 
     mot.cmd=mot_move;
     mot.t_speed=speed; 
     mot.dist=3;
     odo.ran_dist=0;
     return 0;
   }
   else if(mot.finished == 1)
     return mot.finished;
   else
    return 0;
}

int push_the_box(double speed, int time){
  mot.flag_collision = 0;
  switch(mission.sub_m_flag)
  {
    case 0:
      if(follow_line( speed,'c',0, mission.sub_time, sens.cross)){
          mission.sub_m_flag=1;
          mission.sub_time=(-1);
      }
    break;
    case 1:
      if(fwd(3, speed, mission.sub_time, mission.sub_time >= 30)){
          mission.sub_m_flag=2;
          mission.sub_time=(-1);
      }
    break;
    case 2:
      if(fwd(1.4, -1*speed, mission.sub_time, 0)){
          mission.sub_m_flag=3;
          mission.sub_time=(-1);
      }
    break;
    case 3:
      if(turn(-90, speed/2, mission.sub_time)){
          mission.sub_m_flag=4;
          mission.sub_time=(-1);
      }
    break;
    case 11: //To account for nonstraight line when going back
      if(fwd(.10, speed, mission.sub_time, 0)){
          mission.sub_m_flag=4;
          mission.sub_time=(-1);
      }
    break;
    case 4:
      if(follow_line(speed, 'c', 0, mission.sub_time, sens.cross && odo.ran_dist > 0.10)){
          mission.sub_m_flag=5;
          mission.sub_time=(-1);
      }
    break;
    case 5:
      if(fwd(.20, speed, mission.sub_time, 0)){
          mission.sub_m_flag=6;
          mission.sub_time=(-1);
      }
    break;
    case 6:
      if(turn(90, speed/2, mission.sub_time)){
          mission.sub_m_flag=7;
          mission.sub_time=(-1);
      }
    break;
    case 7:
      if(follow_line(speed, 'c', 0, mission.sub_time, sens.cross)){
          mission.sub_m_flag=8;
          mission.sub_time=(-1);
      }
    break;
    case 8:
      if(fwd(.20, speed, mission.sub_time, 0)){
          mission.sub_m_flag=9;
          mission.sub_time=(-1);
      }
    break;
    case 9:
      if(turn(90, speed/2, mission.sub_time)){
          mission.sub_m_flag=10;
          mission.sub_time=(-1);
      }
    break;
    case 10:
      mot.flag_collision = 1;
      if(follow_line(speed, 'c', 0, mission.sub_time, 0/*odo.ran_dist < 2.0*/)){  
        return 1;
      }
    break;
    }
    return 0;
}
int follow_white_line(double speed, double time){
  switch(mission.sub_m_flag)
  {
    case 0:
      if(follow_line(speed,'c',0, mission.sub_time, sens.cross)){//follow line until black cross
          mission.sub_m_flag=1;
          mission.sub_time=(-1);
      }
    break;
    case 1:
      //find the white line
      if(fwd(3, speed, mission.sub_time, sens.w_cross)){//go forward until white cross
          mission.sub_m_flag=2;
          mission.sub_time=(-1);
      }
    break;
    case 2:
      //find the white line
      if(fwd(0.20, speed, mission.sub_time, 0)){//go forward until white cross
          mission.sub_m_flag=3;
          mission.sub_time=(-1);
      }
    break;
    case 3:
      if(turn(90, speed/2, mission.sub_time)){
          mission.sub_m_flag=4;
          mission.sub_time=(-1);
      }
    break;
    case 4:
      //find the white line
      if(follow_line(speed,'c',1, mission.sub_time, sens.cross)){ //follow white until black cross
          return 1;
      }
    break;
  }
  return 0;
}

int gate_loose(double speed, int time){
  double deg;
  double dist = -1;
  int i=0;

  switch(mission.sub_m_flag)
  {
    case 0:
      //if(follow_line(sens.laser[sens.min_las_l]*cos(sens.min_las_l*M_PI/9 + M_PI/18- M_PI/2), speed, 'c', 0, mission.sub_time)){
      if(time == 0){
        dist = sens.laser[sens.min_las_l]*cos(sens.min_las_l*M_PI/9 + M_PI/18- M_PI/2)+robot_length+gate_width/2.0;
        printf("dist: %f\n", dist);
        printf("I sens %f m far in %d\n",sens.laser[sens.min_las_l],sens.min_las_l);
      }
      //fwd(sens.laser[sens.min_las_l]*cos(sens.min_las_l*M_PI/9 + M_PI/18- M_PI/2), speed, mission.sub_time)
      if(fwd(dist, speed, mission.sub_time, 0)){
        printf("I did %f \n",sens.laser[sens.min_las_l]*cos(sens.min_las_l*M_PI/9 + M_PI/18- M_PI/2));
        printf("I sens %f m far in %d\n",sens.laser[sens.min_las_l],sens.min_las_l);
        mission.sub_m_flag=1;
        mission.sub_time=-1;
      }
    break;
    case 1:
      if(turn(90, speed/2, mission.sub_time)){
        printf("I did 90\n");
        mission.sub_m_flag=2;
        mission.sub_time=-1;
      }
    break;
    case 2:
      if (gate_width>=sqrt(pow(sens.laser[sens.min_las],2)+pow(sens.laser[sens.min_las_2],2)-(2*sens.laser[sens.min_las]*sens.laser[sens.min_las_2]*cos((fabs(sens.min_las-sens.min_las_2)+1)*M_PI/9))))
      { 
        sens.min_las_2=sens.min_las_3;
        printf("CH: closest: %d and 3: %d\n",sens.min_las, sens.min_las_2);
      }
      else printf("closest: %d and 2: %d\n",sens.min_las, sens.min_las_2);
      printf("gate_width %f \n",sqrt(pow(sens.laser[sens.min_las],2)+pow(sens.laser[sens.min_las_2],2)-(2*sens.laser[sens.min_las]*sens.laser[sens.min_las_2]*cos((fabs(sens.min_las-sens.min_las_2)+1)*M_PI/9))));
      deg = ((sens.min_las+sens.min_las_2+1)*10 - 90);//
      for(i=0;i<8;i++){
        printf("L%d:%f  ",i, sens.laser[i]);
      }  
      printf("\n ");
      printf("Min:1:%f 2:%f ",sens.laser[sens.min_las], sens.laser[sens.min_las_2]);
      if(turn(deg/2.0, speed/3, mission.sub_time)){
        if(((sens.min_las == 6 && sens.min_las_2 == 2) || (sens.min_las == 2 && sens.min_las_2 == 6)||(sens.min_las == 7 && sens.min_las_2 == 1) || (sens.min_las == 1 && sens.min_las_2 == 7) )&&
          (sens.laser[sens.min_las] >= sens.laser[sens.min_las_2] - 0.1))/* || 
          sens.laser[sens.min_las_l] <= sens.laser[sens.min_las_r] + 0.1)) */ 
        {
          mission.sub_m_flag=4;
          mission.sub_time=-1;
        }
        else{
          mission.sub_m_flag=3;
          mission.sub_time=-1;
        }
      }
    break;
    case 3:
      if(fwd(0.05, speed, mission.sub_time, 0)){
        mission.sub_m_flag=2;
        mission.sub_time=-1;
      }
    break;
    case 4:
      if(fwd(0.750, speed, mission.sub_time, 0))
        return 1;
  }
//    if(gate_passed) return 1; else 
  if(time == 3000) return 1;
  else return 0;
}
int recover_line(int time){
  
   if (sens.min_line_sensor==0)sens.min_line_sensor=1;
   if (sens.min_line_sensor==7)sens.min_line_sensor=6;
   if ((sens.linesensor[sens.min_line_sensor]< BLACK_THRESHOLD && ((sens.linesensor[sens.min_line_sensor+1]< BLACK_THRESHOLD)||
        (sens.linesensor[sens.min_line_sensor-1]< BLACK_THRESHOLD)))){
        sens.lost=0;
        
        return 0;
   }
   if (time==0){ 
     mot.cmd=mot_stop;
   }
   else{ 
     if(turn(90,0.15,time)){
       mot.finished = 0;
        if(turn(-180,0.15,time)){
           mot.finished = 0;
           if(turn(90,0.15,time)){
              mot.finished = 0;
              if(fwd(0.3,-0.2,time, 0))
              mot.finished = 0;
           }
        }
     }
    }   
   
   
   if(mot.finished == 1)
     return mot.finished;
   else
    return 0;
}

int turn(double angle, double speed,int time){
   if (time==0){
     mot.cmd=mot_turn;
     mot.t_speed_t=speed;
     mot.angle=angle*(M_PI/180);
     return 0;
   }
   else
     return mot.finished;
}


void sm_update(smtype *p){
  if (p->state!=p->oldstate){
        printf("State:%d\n", p->state);
       p->sub_time=0;
       p->time=0;
       p->oldstate=p->state;
       p->sub_m_flag = 0;
   }
   else {
     p->time++;
     p->sub_time++;
   }
   
}

void printLog()
{
   int i, j;
   char filename[255];
   char *end = ".dat";
   
   printf("Enter filename: ");
   i = scanf("%s", filename);
   strcat(filename, end);
   FILE *f = fopen(filename, "w");
   if (f == NULL)
   {
      printf("Error opening file!\n");
      exit(1);
   }

   /* print integers and floats */      
   for(j = 0; j < LOGHEIGHT; j++)
   {
	    for(i = 0; i < LOGLENGTH; i++)
	    {
	      fprintf(f, "%f ", dataMatrix.log[j][i]);       
	    }
	  fprintf(f, "\n");
   }

   fclose(f);
}
void myspeed(int aim, motiontype *p, odotype *q, sensetype *s)
{
  double lowdist;
  //if(sqrt(2*0.5*sqrt(pow((q->x)-(p->start_x),2)+pow((q->y)-(p->start_y),2)) )< (p->kf*WHEEL_DIAMETER*p->speed/2))//(p->dist-(p->right_pos+p->left_pos)/2- p->startpos)
  if (p->dist-q->ran_dist > s->ir_min && p->flag_collision)
     lowdist=s->ir_min;
  else
    lowdist=p->dist-sqrt(pow((q->x)-(p->start_x),2)+pow((q->y)-(p->start_y),2));//q->ran_dist;
    
  
  if(sqrt(2*0.5*lowdist)< (p->kf*WHEEL_DIAMETER*p->speed/2))//
    {
    p->speed--;
    
  }
  else{
    if(p->speed>aim){
      p->speed--;
    }
    else if(p->speed<aim){
      p->speed++;
    }
  }
  if (s->ir_min<0.20 && p->flag_collision)
    p->speed=0;
  p->speedcmd=p->speed/127.0;
}
void myspeed_t(int aim, motiontype *p, odotype *q, sensetype *s)
{
  double lowangle;
  //  bad previous formula //if(sqrt(2*0.5*sqrt(pow((q->x)-(p->start_x),2)+pow((q->y)-(p->start_y),2)) )< (p->kf*WHEEL_DIAMETER*p->speed/2))//(p->dist-(p->right_pos+p->left_pos)/2- p->startpos)
  //if (p->dist-q->ran_dist < s->ir_min)
  //   lowdist=p->dist-q->ran_dist;
  //else
  //  lowdist=s->ir_min;
  
  
  lowangle=fabs(p->angle-q->theta);
  if(sqrt(2*0.5*lowangle)< fabs(q->theta-q->old_theta)/0.01)//
  {
    p->speed_t--;
  }
  else{
    if(p->speed_t>aim){
      p->speed_t--;
    }
    else if(p->speed_t<aim){
      p->speed_t++;
    }
  }
  //if (s->ir_min<0.20)
  //  p->speed=0;
  p->speedcmd_t=p->speed_t/127.0;
}

void update_sensors(sensetype *s, odotype *q)
{ 
  int tmp_min_las = 0;
  int tmp_min_las_r = 0;
  int tmp_min_las_l = 5;
  int tmp_min_las_2 = 0;
  int tmp_min_las_3 =0;
  double temp_ir_l = 0;
  double temp_ir_r = 0;
  int lost_counter = 0;
  int line_counter =0;
  int white_line_counter = 0;
  int fork_counter = 0;
  int i = 0;
  int min_line_sensor = 3;
  int max_line_sensor = 3; // for the center on case all sensors are the same
  double sum_i = 0, sum_x_i = 0, sum_in = 0, sum_x_in = 0;
  double Ka[5]={13.26, 18.12, 17.39, 15.00, 13.59};//{0, 15.5488, 14.6827, 14.0957,0 }; optained with matlab
  double Kb[5]={82.15, 87.46, 83.09, 77.13, 61.65};//{1, 82.3226, 81.5651, 68.1602, 1}; optained with matlab
  s->ir_min=4;
  
  /**** Line sensors ****/
  for (i=0; i < LINE_SENS_LENGTH; i++)
  {
    //Read in from sensor and ajust with calibration values
    s->linesensor[i] = linesensor->data[i] * linesensor_interp[0][i] + linesensor_interp[1][i];
    
    if (s->linesensor[i] < s->linesensor[min_line_sensor]){
      min_line_sensor = i;
    }
    if(s->linesensor[i] > s->linesensor[max_line_sensor]){
      max_line_sensor = i;
    }
    sum_i += s->linesensor[i];
    sum_x_i += s->linesensor[i]*i;
    
    //count number of sensors who see black line
    if(s->linesensor[i] < BLACK_THRESHOLD){
      line_counter++;
      if (fork_counter==0||fork_counter==2)
         fork_counter++;
    }
    else{
      if (fork_counter==1)
         fork_counter++;
      if(s->linesensor[i] > BLACK_THRESHOLD && s->linesensor[i] < WHITE_THRESHOLD){
        lost_counter++;
      }
      //count number of sensors who see white line
      else if(s->linesensor[i] > WHITE_THRESHOLD){
        white_line_counter++;
      }
    }
  }
  
  
  if(line_counter == 8 && !s->cross){ 
    cross_counter++;
    s->cross=1;
  }
   if(white_line_counter == 8 && !s->w_cross){ 
    w_cross_counter++;
    s->w_cross=1;
  }
  if(line_counter < 8){ 
    s->cross=0;
  }
   if(white_line_counter < 8){ 
    s->w_cross=0;
  }
  if(lost_counter > 7){
    s->lost = 1;
    //mission.sub_time=mission.time;
    //printf("Help I'm lost... ");
  }
  
  if(fork_counter == 3)
  {
    s->fork_test ++;
    
    if(s->fork_test>2){//5
     if (dataMatrix.counter<LOGLENGTH)
         dataMatrix.log[11][dataMatrix.counter] = 1;
     
      printf("FORK found! \n");
      s->fork = 1;
    }
    
    else
    {
      s->fork=0;
    }
  }
  else
  { 
    if (dataMatrix.counter<LOGLENGTH)
         dataMatrix.log[11][dataMatrix.counter] = 0.8;
    s->fork_test = 0;
    s->fork=0;
  }
  
  
  
  //printf("Cross_counter: %d \n", cross_counter);
  s->min_line_sensor=min_line_sensor;
  s->max_line_sensor=max_line_sensor;
  
  if (min_line_sensor == 0)
  {
    sum_in = s->linesensor[min_line_sensor]+s->linesensor[min_line_sensor+1];
    sum_x_in = s->linesensor[min_line_sensor]*min_line_sensor +
              s->linesensor[min_line_sensor+1]*(min_line_sensor+1);
  }
  else if (min_line_sensor == 7)
  {
    sum_in = s->linesensor[min_line_sensor]+s->linesensor[min_line_sensor-1];
    sum_x_in = s->linesensor[min_line_sensor]*min_line_sensor +
              s->linesensor[min_line_sensor-1]*(min_line_sensor-1);
  }
  else
  {
    sum_in = s->linesensor[min_line_sensor]+s->linesensor[min_line_sensor+1]+s->linesensor[min_line_sensor-1];
    sum_x_in = s->linesensor[min_line_sensor]*min_line_sensor + 
                s->linesensor[min_line_sensor+1]*(min_line_sensor+1) + 
                s->linesensor[min_line_sensor-1]*(min_line_sensor-1);
  }
  if (dataMatrix.counter<LOGLENGTH)
  {
    for(i=0; i < 8; i++)
     dataMatrix.log[i+3][dataMatrix.counter] = s->laser[i];
     
  }
  
  
  /**** IR sensors ****/
  temp_ir_r = s->ir_dist[4];
  temp_ir_l = s->ir_dist[0];
  
  for (i =0; i<IR_SENS_LENGTH-1; i++)
  {
    //Read in from sensor
    s->irsensor[i] = irsensor->data[i];
    
    s->ir_dist[i]=Ka[i]/(s->irsensor[i]-Kb[i]);
    
    //printf("%f \t", s->ir_dist[i]);
    
    if (i >0 && i < IR_SENS_LENGTH-2 && s->ir_dist[i]<s->ir_min && s->ir_dist[i]>0.05) {
      s->ir_min=s->ir_dist[i];
      //printf("Drove: %f m\n",q->x);
      }
    if (dataMatrix.counter<LOGLENGTH)
    {  
      //dataMatrix.log[i+3][dataMatrix.counter] = s->irsensor[i];
    }
  }
  
  if(temp_ir_r != s->ir_dist[4]) s->prev_ir_r=temp_ir_r;
  if(temp_ir_l != s->ir_dist[0]) s->prev_ir_l=temp_ir_l;
  
  //printf("ir:%f pre_ir:%f\n", s->ir_dist[4],s->prev_ir_r);
  
  /**** Laser ****/
  for (i =0; i<9; i++)
  { 
    if (laserpar[8-i]>3.5)//for real life i=[8-i]
      s->laser[i]=3.5;
    else{
      s->laser[i]=laserpar[8-i];//for real life i=[8-i]
    }
    
    if(i < 8 && i > 0 && s->laser[i] < s->laser[tmp_min_las_3]){
      if(s->laser[i] < s->laser[tmp_min_las_2]){
        if(s->laser[i] < s->laser[tmp_min_las]){
          tmp_min_las_3 = tmp_min_las_2;
          tmp_min_las_2 = tmp_min_las;
          tmp_min_las = i;
        }
        else{
        tmp_min_las_3 = tmp_min_las_2;
        tmp_min_las_2 = i;
        }
      }
      else{
        tmp_min_las_3=i; 
      }
    }
    if(i > 4 && i < 8 && s->laser[i] < s->laser[tmp_min_las_l]) tmp_min_las_l = i;
    if(i < 4 && i > 0 && s->laser[i] < s->laser[tmp_min_las_r]) tmp_min_las_r = i;
    
  }
  
  s->min_las_l = tmp_min_las_l;
  s->min_las_r = tmp_min_las_r;
  s->min_las = tmp_min_las;
  s->min_las_2 = tmp_min_las_2;
  s->min_las_3 = tmp_min_las_3;
  //printf("min_las_l: %d has: %f\n", tmp_min_las_l, s->laser[s->min_las_l]);
  
}
void update_map(odotype *q, sensetype *s ,maptype *m){
  int i,j,k,n,sign;
  m->theta=0.0;//M_PI/4;
  //m->theta=atan2(m->y-m->old_y,m->x-m->old_x); aplly later
  if (!(m->map[m->x][m->y]%3==0))
      m->map[m->x][m->y]*=3;
  if (!(m->map[m->xg][m->yg]%7==0))
      m->map[m->xg][m->yg]*=7;
  
  
  //MAP: -1=unkown, positive=known and open; even = accesible; x3= visited; x5=current closens accesible to the goal. x7= goal
  for(sign=-1;sign<2; sign=sign+2){
    for(i=0;i<m->laser_max/m->cell;i++){
      for(j=0;j<m->laser_max/m->cell;j++){
        for(n=0;n<9;n++){
          
          if(m->map[m->x+i*sign][m->y+j*sign]<0){
            if(atan2(j*sign,i*sign)>=m->alpha*n+m->theta && atan2(j*sign,i*sign)<m->alpha*(n+1)+m->theta){
              if(sqrt(i*i+j*j)*m->cell<s->laser[n]){
                m->map[m->x+i*sign][m->y+j*sign]=m->map[m->x+i*sign][m->y+j*sign]*-2;//*-1 to say it was discovered and in the map. *2 to put it in the accesible points
                if (pow(m->xg-m->x+i*sign,2)+pow(m->yg-m->y+j*sign,2)<((m->xg-m->xcg)*(m->xg-m->xcg)+(m->xg-m->xcg)*(m->xg-m->xcg))){
                  m->map[m->x+i*sign][m->y+j*sign]=(m->map[m->x+i*sign][m->y+j*sign])*5;
                  m->map[m->xcg][m->ycg]=m->map[m->xcg][m->ycg]/5;
                  m->xcg=m->x+i*sign;
                  m->ycg=m->y+j*sign;
                }
              }
              else if(sqrt(i*i+j*j)<m->laser_max){
                //neighnbourghs
                for (k=0;k<2;k++){
                  if(m->map[m->x+i*sign+k][m->y+j*sign]%2==0)
                   m->map[m->x+i*sign+k][m->y+j*sign]/=2;//so it doesn belong to the accesible points anymore
                  if(m->map[m->x+i*sign-k][m->y+j*sign]%2==0)
                   m->map[m->x+i*sign-k][m->y+j*sign]/=2;
                   
                  if(m->map[m->x+i*sign][m->y+j*sign+k]%2==0)
                   m->map[m->x+i*sign][m->y+j*sign+k]/=2;//so it doesn belong to the accesible points anymore
                  if(m->map[m->x+i*sign][m->y+j*sign-k]%2==0)
                   m->map[m->x+i*sign][m->y+j*sign-k]/=2;
                   
                }
                
                if(m->map[m->x+i*sign-1][m->y+j*sign-1]%2==0)
                   m->map[m->x+i*sign-1][m->y+j*sign-1]/=2;
                if(m->map[m->x+i*sign-1][m->y+j*sign+1]%2==0)
                   m->map[m->x+i*sign-1][m->y+j*sign+1]/=2;
                if(m->map[m->x+i*sign+1][m->y+j*sign-1]%2==0)
                   m->map[m->x+i*sign+1][m->y+j*sign-1]/=2;
                if(m->map[m->x+i*sign+1][m->y+j*sign+1]%2==0)
                   m->map[m->x+i*sign+1][m->y+j*sign+1]/=2;
                   
              }
            }
          }
        }
      }
    }
   }/*
   for(k=0;k<9; k++){
    printf("%f ",s->laser[k]);
   }
   printf("\n");
   for(j=0;j<25; j++){
    
    for(i=0;i<25; i++){
        printf("%3d ",m->map[i][j]);
    }
    printf("\n");
   }
   printf("\n \n");*/
}


