#include "Motor_real.h"
#include "Balanced_real.h"
#include "KalmanFilter.h"

unsigned long t0=0;
#include "sbus.h"
#define NUM_OF_CHANNEL  16
extern Motor Motor;

Timer2 Timer2;
extern Mpu6050 Mpu6050;
extern Balanced Balanced;
extern KalmanFilter kalmanfilter;


int mode=0;
bool arret_moteur=false;
bool stopped=false;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
//bfs::SbusTx sbus_tx(&Serial3);
/* SBUS data */
bfs::SbusData data;

//la channel envoie un nombre sur la arduino, on veut que ce nombre soit entre 0 et 255 (il est initialement entre -1000 et 1000 ou un truc comme ça)
int convert_tension(float tension_read){//pour convertir le signal reçu de la channel 
  int converted=int(0.15458937198067632850*tension_read - 26.589371980676328502);
  return min(converted,255);//si on dépasse 255, ça vaut 255 quand même. 
}

int convert_coef(float tension_read){//pour convertir le signal reçu de la channel 
  float converted=(0.15458937198067632850*tension_read - 26.589371980676328502);
  converted=max(0,converted);
  return min(converted,255);//si on dépasse 255, ça vaut 255 quand même. 
}

//on bidouille la plage de valeurs de la vitesse des roues (pour qu'elle soit plutôt entre 5 et 100 par exemple, 255 c'est trop)
int convert_channel(int value_channel){
  
  int converted=((value_channel-128));
  
  return converted;
}

float convert_speed(float tension_spd_read){
  int converted=convert_tension(tension_spd_read);
  converted=float((converted/255)*1.5);
  return converted;
}

int find_mode(int command){
  if (command<200){
    return 0;
  }
  if (command<1000){
    return 1;
  }
  if (command>1000){
    return 2;
  }
  
}

void updt_offset(float offset)
{
  Balanced.offset_orientation=offset*10/255;
  return;

}

void updt_coef(int command, float kp, float kd){
  if (command<200){
    Balanced.kp_balance=kp*30/255;
    Balanced.kd_balance=kd*10/255;
    return;
  }
  if (command<1000){
    Balanced.kp_speed=kp*50/255 ;
    Balanced.ki_speed=kd*15/255;
    return;
  }
  if (command>1000){
    Balanced.kp_turn=kp*0.3/255; //0.13 est la bonne valeur pour la vide
    Balanced.kd_turn=kd*10/255;
  }
}
void reset_coef(){
  Balanced.kp_balance = 0.0; //valeur trouvée empiriquement 6.5
  Balanced.kd_balance = 0.0; //valeur trouvée empiriquement 0.2
  Balanced.kp_speed = 0; 
  Balanced.ki_speed = 0.;
  Balanced.kp_turn = 0 ;
  Balanced.kd_turn = 0.;
  
}

void telecom(){
  if (sbus_rx.Read()) {

    //Grab the received data 
    data = sbus_rx.data();

    bool btn_kp=data.ch[4]>1000;//quand le bouton est vers le bas, la valeur vaut 170 et quand le bouton est vers le haut ça vaut 1170 (ou un truc comme ça)
    int reset_ki=data.ch[1];
    int channel6=data.ch[6];
    int channel7=data.ch[7];
    int channel1=data.ch[0];
    float converted_kp=convert_coef(channel6);
    float converted_kd=convert_coef(channel7);
    float converted_offset=convert_coef(channel1);
    
    int channel8=data.ch[8];
    int channel9=data.ch[9];
    mode=find_mode(channel9);
    
    int channel3=data.ch[3];
    // Serial.println("mode: "+String(mode));
    if (not arret_moteur){
      if (not stopped){
        
        // bool stopped=data.ch[5]<1000;
        
        // // Display the received data       
        // int channel1=data.ch[1];
        
        // int converted_forward=convert_tension(channel1);
        
        // converted_forward=convert_channel(converted_forward);

        // Balanced.Speed_control(converted_forward,0);
        
      }
      else{
        Balanced.Stop();
      }
    }
    else{
      Motor.Stop();
    }

    if (btn_kp){
        updt_coef(channel8,converted_kp,converted_kd); //changement des coefficient   
        updt_offset(converted_offset);
    }
    if (channel3>1200){
       reset_coef(); //reset des coefficient
    }
    if (reset_ki>1200){
      Balanced.car_speed_integeral=0.0;
      
    }
      
    if ( millis() - t0 > 1000){
        // Serial.print("angle: ");
        // Serial.println(kalmanfilter.angle);

        // Serial.print("kp_balance: ");
        // Serial.println(Balanced.kp_balance);
        // Serial.print("kd_balance: ");
        // Serial.println(Balanced.kd_balance);
        // Serial.print("kp_speed: ");
        // Serial.println(Balanced.kp_speed);
        // Serial.print("ki_speed: ");
        // Serial.println(Balanced.ki_speed);
        // Serial.print("car integral: ");
        // Serial.println(Balanced.car_speed_integeral);
        // Serial.print("kp_turn: ");
        // Serial.println(Balanced.kp_turn);
        // Serial.print("kd_turn: ");
        // Serial.println(Balanced.kd_turn);
        // Serial.print("offset_orientation: ");
        // Serial.println(Balanced.offset_orientation);

        // Serial.println("----------------------------\r");
        //Serial.println(converted_forward);
        
        t0=millis();
      }
   if (mode==0){
    arret_moteur=true;
    //Serial.println("MODE: NO MOTOR");
   }
   if (mode==1){
    arret_moteur=false;
    stopped=true;
    //Serial.println("MODE: STOPPED");
   }
   if (mode==2){
    arret_moteur=false;
    stopped=false;
    //Serial.println("MODE: GO");
   }
  }
}

void Timer2::init(int time)
{
  MsTimer2::set(time,interrupt);
  MsTimer2::start();
  //Balanced.test_interrupt=2;
  
}

static void Timer2::interrupt()
{ 
  sei(); //enable the global interrupt
  //Balanced.test_interrupt=2;
  Balanced.Get_EncoderSpeed();//on récupère les données des encodeurs
  Mpu6050.DataProcessing();//on récupère les données de la centrale inertielle

  //Serial.print("angle apres Kalman: ");
  //Serial.println(kalmanfilter.angle);

  //Kp_balanced = 6.5;
  //Kd_balanced = 

  Balanced.PD_VerticalRing();
  Balanced.interrupt_cnt++;
  if(Balanced.interrupt_cnt > 8)//toutes les 8 itérations
  {
    Balanced.interrupt_cnt=0;
    Balanced.PI_SpeedRing();//calcul de 
    Balanced.PI_SteeringRing();
  }
  telecom();
  if (not arret_moteur){
    Balanced.Total_Control();
  }
  //Serial.println("----------------------------\r");
}



void setup() {
  //Serial for the receiver
  Timer2.init(10);
  Mpu6050.init();
  Serial1.begin(9600);
  //Serial monitor
  Serial.begin(9600);

  //motors
  Serial2.begin(115200);
   //1 motor channel 4 nombre de pôles
  
  while (!Serial1) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  //sbus_tx.Begin();
  pinMode(19,  INPUT);//receiver is on pin RX1=19

}



void loop() {
  static unsigned long print_time;
  
  for(int i=0;i<5;)
  {
      if(millis() - print_time > 3000)
   { 
      print_time = millis();
      Serial.println(Balanced.test_interrupt);
   }
  }
}

