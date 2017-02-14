
/* CACC Cristina Nita-Rotaru Nov. 5 2016 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#define max(a, b) ((a) > (b) ? (a) : (b))

#define MAX_BUF 1000

/* how often does the task execute 10 ms
   what is the minimum delay for the pakets  5ms
   periodicity of sending the message 15, 16 ms or 20 ms
   time from decision to actuation 0 - 5 ms
   maximum acceleration 0.3G 1.5G
   maxium decelleratio 0.5 to 1.5
   maximum decelleration pre-crash 0.2 - 0.5 G
   the sensor can also be periodic  8, 10, 16, 20, 40 50 ms, multiple of 8 or 10
   how does the sensor know the speed of the leading car; speed is sensed
   leader change not considered; a car always follows the leader; if there is nobody in front the car does not do CACC. */


#define SIM_TIME   300  // simulation time in seconds; set to 5 minutes 
#define TIME_UNIT  0.01 // time unit when the cacc is performed; in seconds; try 0.01
#define MAX_POINTS 1000 /* 3000 */

#define MAX_VELOCITY 20    /* 85 miles per hour */
#define SENSOR_COVERAGE 50 /* m   */

const double SAFETIMEGAP =  0.55; /* seconds */
const unsigned int  CAR_LENGTH = 5; /* m */
const double MAX_DECEL = 5; /* m2/s */
const double MIN_ACC = 1; /*  m2/s */

enum A_Mode {Cacc_ca, Cacc_gc, Acc};

#define ERR_COLLISION           -2
#define ERR_NO_CARLENGTH        -3
#define ERR_NOMAXDECL           -4
#define ERR_NEG_SAFEGAP         -5
#define ERR_COLLISION_AVOIDANCE -6

struct Point{
  double d; /* Distance from origin for car in m */
  double v; /* velocity of the car m/a */
  double a; /* acceleration of the car m2/s */
  enum A_Mode  mode; /* acc computation, cacc or acc or ca */
};

struct Car{
  struct Point p[MAX_POINTS]; /* trajectory */
  double maxda; /* max deceleration of the car m2/s */
  unsigned int length; /* length of car in m */
};


void init_point(struct Point * p, double d, double v, double a, enum A_Mode  m) {
  p->d = d;
  p->v = v;
  p->a = a;
  p->mode = m;
}

void init_car(struct Car * c) {
  c->maxda = MAX_DECEL;
  c->length = CAR_LENGTH;
  
  for(int i=0; i<MAX_POINTS; i++) {
    init_point(&(c->p[i]), 0.0, 0.0, 0.0, Cacc_gc);
  }
}

void update_point_constant_speed(struct Car *c, double t, int i) {
  assert(i>0);

  c->p[i].a = c->p[i-1].a;
  c->p[i].v = c->p[i-1].v;
  c->p[i].d = c->p[i-1].d + c->p[i-1].v*t;
}


void update_point_changed_speed(struct Car *c, double t, int i) {
  assert(i>0);
  
  c->p[i].a = c->p[i-1].a;
  c->p[i].v = c->p[i-1].v + c->p[i-1].a*t;
  if(c->p[i].v > MAX_VELOCITY) {
    c->p[i].v = MAX_VELOCITY;
  }
  c->p[i].d = c->p[i-1].d + c->p[i-1].v*t + 0.5 * c->p[i-1].a*t*t;
}

double compute_gap(struct Car *r, struct Car *f, int i) {
  assert(r->length > 0);
  return r->p[i].d - f->p[i].d - r->length;
}

double compute_safegap(struct Car *r, struct Car *f, int i) {
  assert(f->maxda != 0);
  assert(r->maxda != 0);
  
  return max( (0.1)*f->p[i].v + (f->p[i].v*f->p[i].v) / (2 * f->maxda) - (r->p[i].v*r->p[i].v) / (2 * r->maxda) + 1.0, 1);
}


int compute_acc(struct Car *r, struct Car *f, double safetime, int i) { /* returns the new acceleration in newacc */
  double gap;
  double safegap;
  int ret = 0;
  
  gap = compute_gap(r, f, i);
  if (gap < 0) {
    fprintf( stderr, "Warning: The following car is ahead of the leader or has already collided\n");
    ret =  ERR_COLLISION;
    goto end;
  }
  
  safegap = compute_safegap(r, f, i);
  if (safegap < 0) {
    fprintf(stderr, "Safegap %f is negative \n", safegap);
    ret = ERR_NEG_SAFEGAP;
    goto end;
  }
  
  if (gap < safegap) {
    f->p[i].mode = Cacc_ca;
    fprintf(stderr, " Currentgap %4.2f is less than safegap %4.2f, entering ca with max deceleration %4.2f\n", gap, safegap, f->maxda);
    f->p[i].a = -MAX_DECEL;
    ret = ERR_COLLISION_AVOIDANCE;
  }
  else {
    f->p[i].mode = Cacc_gc;
    double desiredaccel = 0.66 * r->p[i].a + 0.99 * (r->p[i].v - f->p[i].v) + 4.08 * (gap - f->p[i].v*SAFETIMEGAP - 2.0);
    double accelcontrol = (desiredaccel - f->p[i].a) / 0.4 * 0.1 + f->p[i].a;
    accelcontrol = accelcontrol < -3 ? -3 : accelcontrol;
    accelcontrol = accelcontrol > 3 ? 3 : accelcontrol;
    f->p[i].a = accelcontrol;
    ret = 0; 
  }
 end:
  return ret;
}


double getvalue(const char * prompt) {
  double x;
  printf("%s", prompt);
  scanf("%lf", &x);
  
  return x;
}

void getCarStartPoint(struct Car *c) {
  c->p[0].d = getvalue("Distance: ");
  c->p[0].v = getvalue("Velocity: ");
  c->p[0].a = getvalue("Acceleration: ");
}

void Print_Params() {
  printf("SafetimeGap is %4.2f\n", SAFETIMEGAP);
  printf("Car lenthg is %d\n", CAR_LENGTH);
  printf("Max deceleration is %4.2f\n", MAX_DECEL);
  printf("Max velocity is %d\n", MAX_VELOCITY);
  printf("Time unit is %4.2f\n", TIME_UNIT);
  printf("Trajectory points num is %d\n",  MAX_POINTS);
}

void write_trajectory_json(struct Car* c, char* filename, double t_unit) {
  FILE *outfile = NULL;

  outfile = fopen(filename,"w+"); 
  if(outfile == NULL) {
    perror("file open failed");
    exit(1);
  }
			  
  fputs("\"time\":\"", outfile);
  for(int i=0; i<MAX_POINTS; i++) {
    float d = t_unit*i;
    char buf[10];
    
    sprintf(buf, "%3.2f ", d);
    fputs(buf, outfile);
  }
  fputs("\",", outfile);

  
  fputs("\"speed\":\"", outfile);
  for(int i=0; i<MAX_POINTS; i++) {
    float d = c->p[i].v;
    char buf[10];

    sprintf(buf, "%4.2f ", d);
    fputs(buf, outfile);
  }
  fputs("\",", outfile);

  
  fputs("\"location\":\"", outfile);
  for(int i=0; i<MAX_POINTS; i++) {
    float d = c->p[i].d;
    char buf[10];
    
    sprintf(buf, "%4.2f ", d);
    fputs(buf, outfile);
  }
  fputs("\"\n", outfile);
  
  fclose(outfile);
}


int main() {
  struct Car carA, carB, carC;
  double safetime = SAFETIMEGAP;
  double t_unit = TIME_UNIT; //seconds
  double olda;
  double oldv;
  int ret;

  
  printf("--------------------------\n");
  printf("CACC implementaLeadertion in C\n");
  Print_Params();
  printf("--------------------------\n");
    
  printf("\n\nEnter cars information \n\n");

  printf("Car A info\n");
  init_car(&carA);
  getCarStartPoint(&carA);
  
  printf("Car B info\n");
  init_car(&carB);
  getCarStartPoint(&carB);

  /*printf("Car C info\n");
  init_car(&carC);
  getCarStartPoint(&carC);*/

  ret = compute_acc(&carA, &carB, safetime, 0); 
  if((ret ==  ERR_COLLISION) || (ret == ERR_NEG_SAFEGAP) || (ret == ERR_COLLISION_AVOIDANCE)) {
    fprintf(stderr, "Follower ahead of leader or in collision avoidance\n");
    goto end;
  }
  
  printf("time: %3.2f  CarA   d = %4.2f  v = %4.2f   a = %4.2f\n", 0.0, carA.p[0].d, carA.p[0].v, carA.p[0].a);
  printf("time: %3.2f  CarB   d = %4.2f  v = %4.2f   a = %4.2f\n", 0.0, carB.p[0].d, carB.p[0].v, carB.p[0].a);
    
  for(int i = 1; i< MAX_POINTS; i++) {
    update_point_constant_speed(&carA, t_unit, i); //leader, a and v stayed the same
    update_point_changed_speed(&carB, t_unit, i); // acc changed for this time interval
    
    printf("time: %3.2f  CarA  d = %4.2f  v = %4.2f  a = %4.2f  \n", t_unit*i, carA.p[i].d, carA.p[i].v, carA.p[i].a);
    printf("time: %3.2f  CarB  d = %4.2f  v = %4.2f  a = %4.2f  \n", t_unit*i, carB.p[i].d, carB.p[i].v, carB.p[i].a);
    
    ret = compute_acc(&carA, &carB, safetime, i);
    if((ret == ERR_COLLISION) || /*(ret == ERR_NEG_SAFEGAP) ||*/ (ret == ERR_NO_CARLENGTH) || (ret == ERR_NOMAXDECL))  {
      goto end; //collision
    }
    
  }
  write_trajectory_json(&carA, "CarA", t_unit);
  write_trajectory_json(&carB, "CarB", t_unit);
  
 end:    
  return 0;
}

