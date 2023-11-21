#include <18F4550.h>
#device *=16 adc=10
#include <math.h>// include thu vien toan hoc
#FUSES NOWDT, HS
#use delay(clock=20m)
/*
   wall data
   7 6 5 4 3 2 1 0
   0|0|0|0|W|S|E|N
*/
// dinh nghia dia chi BIT trong 1 byte data
#define BIT0   0x01 // 0000 000'0' BIT0 luu trang thai buc tuong phia N,
#define BIT1   0x02 // 0000 00'0'0 BIT1 Luu trang thai buc tuong phia E
#define BIT2   0x04 // 0000 0'0'00 BIT2 luu trang thai buc tuong phia S
#define BIT3   0x08 // 0000 '0'000 BIT3 luu trang thai buc tuong phia W
#define BIT4   0x10 // 000'0' 0000
#define BIT5   0x20 // 00'0'0 0000
#define BIT6   0x40 // 0'0'00 0000
#define BIT7   0x80 // '0'000 0000

#byte PORTE=0xF84
#bit   LED= PORTE.2
#bit   EN_MOTOR_L = PORTE.0
#bit   EN_MOTOR_R = PORTE.1
#byte PORTB=0xF81
#BIT   LED_L = PORTB.3
#bit   LED_F= PORTB.4
#BIT   LED_R = PORTB.5
#byte PORTC=0xF82
#BIT   DIR_L = PORTC.0
#BIT   STEP_L = PORTC.1
#BIT   STEP_R = PORTC.2
#byte PORTD=0xF83
#BIT   DIR_R = PORTD.3
#BIT   SPEAKER = PORTD.7


#define CHECKBIT(x,b)   (x&b) // check bit "b" in byte "x"
// return 1 if b=high, 0 if b=low

unsigned char mazeflood[36]={ // mang luu tru gia tri flood fill
6, 5, 4, 3, 4, 5,
5, 4, 3, 2, 3, 4,
4, 3, 2, 1, 2, 3,
3, 2, 1, 0, 1, 2,
4, 3, 2, 1, 2, 3,
5, 4, 3, 2, 3, 4
};
unsigned char current_cell = 0; // dai dien cho vi tri chuot dang dung
unsigned char walldata[36];

/*
         7 6 5 4 3 2 1 0
   dir: |0|W |0 |S|0|E|0|N|
           64   16   4   1
   ban dau gan gia tri cho huong hien tai cua chuot la huong N tuong ung voi dir = 1 (gia tri thap phan)
*/
unsigned char current_dir=1; // huong hien tai cua chuot
unsigned char start=0,count_timer0=0;
unsigned int16 led_right=0,led_front=0,led_left=0;
unsigned int16  left_on=0,right_on=0,right=0,left=0,front_l=0,front_r=0,front_r_on=0,front_l_on=0;
unsigned int16 right_off=0,left_off=0,front_l_off=0,front_r_off=0;
unsigned int16  error_l=0,error_r=0,error_front_r=0,error_front_l=0;
void flood_fill(unsigned char);
unsigned char destination; // o muc tieu can den
unsigned char step(void);
unsigned int16 delay; // toc do chay
unsigned char speed; // bien luu so lan chay
int16 timer0set=65436;
// 
#int_timer0
void ngat_timer0()
{
set_timer0(timer0set);
count_timer0++;
   if (count_timer0==1)
      {
         set_adc_channel(0);
         left_off=read_adc();
         set_adc_channel(3); 
         right_off=read_adc();
         set_adc_channel(1);
         front_l_off=read_adc();
         set_adc_channel(2);
         front_r_off=read_adc();
         LED_L=1;
         LED_R=1;
         LED_F=1;
      }
   if (count_timer0==2)
      {
         set_adc_channel(0); 
         left_on=read_adc();
         set_adc_channel(3); 
         right_on=read_adc();
         set_adc_channel(1);
         front_l_on=read_adc();
         set_adc_channel(2);
         front_r_on=read_adc();
         LED_L=0;
         LED_R=0;
         LED_F=0;
         count_timer0=0;
   left=left_on-left_off;
   right=right_on-right_off;
   front_l=front_l_on-front_l_off;
   front_r=front_r_on-front_r_off;
   if (left>=right)
   {
      error_l=left-right; // lech trai
      error_r=0;
   }
   else 
   {
      error_r=right-left; // lech phai
      error_l=0;
   }
   if (front_l>=front_r) 
   {
      error_front_l=front_l-front_r; // lech trai
      error_front_r=0;
   }
   else
   {
      error_front_r=front_r-front_l; // lech trai
      error_front_l=0;
   }
      }
}
#int_EXT2
void ngat_timer2()
{
   if (!(input(pin_b2)))
   {
      start=1;
   }
}
void speed_run();
void turnleft();
void turnright();
void movestraight();
void flood_fill();
void read_sensor();
//////////////////////////////////////////////////////////////////////////// main
void main()
{
   unsigned char wallstring;
   timer0set=65436;
   setup_adc_ports(AN0_TO_AN3|VSS_VDD);//chon kenh ADC 0->3, vref=vdd 0-5V
   setup_adc(ADC_CLOCK_INTERNAL);//dinh thoi gian lay mau = xung clock
   output_b(0x00);
   output_c(0x00);
   output_d(0x00);
   output_e(0x00);
      enable_interrupts(global);
   // ngat timer0 div/256
   /*
   f=20mhz/4, => T (timer0) = 4*256/20M = 0.0512ms
   */
   setup_timer_0(87); // 10000111: enable (bit7), LSB = 0111 =7 =f(timer)/256
   enable_interrupts(int_timer0);
   //set_timer0(64756);// dat gia tri bo dem tiemr0
   // ngat ngoai timer2
   enable_interrupts(int_EXT2);
   ext_int_edge ( 2 , H_TO_L );
   start=0;
   unsigned char steper;         //Temporarily holds the movement command
   walldata[0]=142;
   walldata[1]=12;
   walldata[2]=4;
   walldata[3]=4;
   walldata[4]=4;
   walldata[5]=6;
   walldata[6]=8;
   walldata[12]=8;
   walldata[18]=8;
   walldata[24]=8;
   walldata[30]=9;
   walldata[11]=2;
   walldata[17]=2;
   walldata[23]=2;
   walldata[29]=2;
   walldata[35]=3;
   walldata[31]=1;
   walldata[32]=1;
   walldata[33]=1;
   walldata[34]=1;   
   delay=500;   
while (true)
{
led=1;
   if (start==1&&front_r>=150)
   {
   led=0;
   delay_ms(400);
   led=1;
   movestraight(); 
   if (current_dir==1) current_cell+=6; // huong North
   if (current_dir==4) current_cell+=1; // huong East
   if (current_dir==16) current_cell-=6; // huong South
   if (current_dir==64) current_cell-=1; // huong West
   destination=21;
   while (true)
      {
        if (speed<2)
        {
      read_sensor();
         if (current_dir==1)
         {
               // cap nhat buc tuong cell hien tai
               if (!(checkbit(current_cell,bit7)))
               {
              wallstring=((1<<7)|(led_left<<3)|(0<<2)|(led_right<<1)|(led_front));
              walldata[current_cell]=wallstring;
               }
              // cap nhat thong tin cho buc tuong cell hang xom
              if ((!(checkbit(walldata[current_cell+1],bit7)))&&(!(current_cell==5))&&(!(current_cell==11))
                  &&(!(current_cell==17))&&(!(current_cell==23))&&(!(current_cell==29))&&(!(current_cell==35))&&(led_right==1))
                  walldata[current_cell+1] = ((walldata[current_cell+1])|(1<<3));
              if ((!(checkbit(walldata[current_cell+6],bit7)))&&(!(current_cell==30))&&(!(current_cell==31))
                  &&(!(current_cell==32))&&(!(current_cell==33))&&(!(current_cell==34))&&(!(current_cell==35))&&(led_front==1))
                  walldata[current_cell+6] = ((walldata[current_cell+6])|(1<<2));
              if ((!(checkbit(walldata[current_cell-1],bit7)))&&(!(current_cell==0))&&(!(current_cell==6))
                  &&(!(current_cell==12))&&(!(current_cell==18))&&(!(current_cell==24))&&(!(current_cell==30))&&(led_left==1))
                  walldata[current_cell-1] = ((walldata[current_cell-1])|(1<<1));        
                             
         }
         // East
         if (current_dir==4)
         {
                        if (!(checkbit(current_cell,bit7)))
               {
               wallstring=((1<<7)|(0<<3)|(led_right<<2)|(led_front<<1)|(led_left));
               walldata[current_cell]=wallstring; 
               }
              if ((!(checkbit(walldata[current_cell+1],bit7)))&&(!(current_cell==5))&&(!(current_cell==11))
                  &&(!(current_cell==17))&&(!(current_cell==23))&&(!(current_cell==29))&&(!(current_cell==35))&&(led_front==1))
                  walldata[current_cell+1] = ((walldata[current_cell+1])|(1<<3));
              if ((!(checkbit(walldata[current_cell+6],bit7)))&&(!(current_cell==30))&&(!(current_cell==31))
                  &&(!(current_cell==32))&&(!(current_cell==33))&&(!(current_cell==34))&&(!(current_cell==35))&&(led_left==1))
                  walldata[current_cell+6] = ((walldata[current_cell+6])|(1<<2));
              if ((!(checkbit(walldata[current_cell-6],bit7)))&&(!(current_cell==0))&&(!(current_cell==1))
                  &&(!(current_cell==2))&&(!(current_cell==3))&&(!(current_cell==4))&&(!(current_cell==5))&&(led_right==1))
                  walldata[current_cell-6] = ((walldata[current_cell-6])|(1));             
         }
         // South
         if (current_dir==16)
         {
                        if (!(checkbit(current_cell,bit7)))
               {
               wallstring=((1<<7)|(led_right<<3)|(led_front<<2)|(led_left<<1)|(0));
               walldata[current_cell]=wallstring; 
               }
              if ((!(checkbit(walldata[current_cell+1],bit7)))&&(!(current_cell==5))&&(!(current_cell==11))
                  &&(!(current_cell==17))&&(!(current_cell==23))&&(!(current_cell==29))&&(!(current_cell==35))&&(led_left==1))
                  walldata[current_cell+1] = ((walldata[current_cell+1])|(1<<3));
              if ((!(checkbit(walldata[current_cell-6],bit7)))&&(!(current_cell==0))&&(!(current_cell==1))
                  &&(!(current_cell==2))&&(!(current_cell==3))&&(!(current_cell==4))&&(!(current_cell==5))&&(led_front==1))
                  walldata[current_cell-6] = ((walldata[current_cell-6])|(1));
              if ((!(checkbit(walldata[current_cell-1],bit7)))&&(!(current_cell==0))&&(!(current_cell==6))
                  &&(!(current_cell==12))&&(!(current_cell==18))&&(!(current_cell==24))&&(!(current_cell==30))&&(led_right==1))
                  walldata[current_cell-1] = ((walldata[current_cell-1])|(1<<1));             
         }
         // West
         if (current_dir==64)
         {
                        if (!(checkbit(current_cell,bit7)))
               {
               wallstring=((1<<7)|(led_front<<3)|(led_left<<2)|(0<<1)|(led_right));
               walldata[current_cell]=wallstring;   
               }
              if ((!(checkbit(walldata[current_cell-1],bit7)))&&(!(current_cell==0))&&(!(current_cell==6))
                  &&(!(current_cell==12))&&(!(current_cell==18))&&(!(current_cell==24))&&(!(current_cell==30))&&(led_front==1))
                  walldata[current_cell-1] = ((walldata[current_cell-1])|(1<<1));
              if ((!(checkbit(walldata[current_cell+6],bit7)))&&(!(current_cell==30))&&(!(current_cell==31))
                  &&(!(current_cell==32))&&(!(current_cell==33))&&(!(current_cell==34))&&(!(current_cell==35))&&(led_right==1))
                  walldata[current_cell+6] = ((walldata[current_cell+6])|(1<<2));
              if ((!(checkbit(walldata[current_cell-6],bit7)))&&(!(current_cell==0))&&(!(current_cell==1))
                  &&(!(current_cell==2))&&(!(current_cell==3))&&(!(current_cell==4))&&(!(current_cell==5))&&(led_left==1))
                  walldata[current_cell-6] = ((walldata[current_cell-6])|(1)); 
           }   
        }
      flood_fill();
      led=1;
       steper = step();
      //////////// move
      if (steper== 'R')
      {
         turnright();
         current_dir *= 4;
         if (current_dir == 0)   current_dir = 1;     
      }
      else if (steper=='L')
      {
         turnleft();
         current_dir /= 4; 
         if (current_dir == 0)   current_dir = 64;
      }
      else if (steper=='F')
      {
         turnleft();
         current_dir *= 4; 
         if (current_dir == 0)   current_dir = 1;
         turnleft();
         current_dir *= 4;
         if (current_dir == 0)   current_dir = 1;      
      }      
      movestraight();
      if (current_dir==1) current_cell+=6; // huong North
      if (current_dir==4) current_cell+=1; // huong East
      if (current_dir==16) current_cell-=6; // huong South
      if (current_dir==64) current_cell-=1; // huong West
      led=0;
 if (destination==0&&current_cell==0) speed_run();
 if (current_cell==21)
               {
                  speaker=1;
                  delay_ms(40);
                  speaker=0;
                  delay_ms(40);
                  unsigned char mazeflood[36]={ 
                  0, 1, 2, 3, 4, 5,
                  1, 2, 3, 4, 5, 6,
                  2, 3, 4, 5, 6, 7,
                  3, 4, 5, 6, 7, 8,
                  4, 5, 6, 7, 8, 9,
                  5, 6, 7, 8, 9, 10
                  };
                  destination = 0;
                  if (speed>2) delay=280;
               }
      }
   }
}
}
/// sensor
void read_sensor()
{
// save wall data
   if (left>80) led_left=1;
   else led_left=0; 
   if (right>80) led_right=1;
   else led_right=0;
   if ((front_l>100)||(front_r>100)) led_front=1;
   else led_front=0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flood_fill()
{
   unsigned char neighbour_val[] = {255,255,255,255}; 
   unsigned char wallinfo = 0, x = 0;
   int stk[128], stkptr = 0, stk_empty_flag = 0;   
   unsigned char floodcell = 0;
   stk[0] = 255;
   stkptr++; 
   stk[stkptr] = current_cell; 
   while (stk_empty_flag == 0) 
   {

      floodcell = stk[stkptr];

      if (stkptr == 1) stk_empty_flag = 1;  
      else 
      stkptr--;

      neighbour_val[0] = 255;   
      neighbour_val[1] = 255;   
      neighbour_val[2] = 255;   
      neighbour_val[3] = 255;  

      wallinfo = walldata[floodcell];

      if (!(CHECKBIT(wallinfo, BIT0) ) )  
         neighbour_val[0] = mazeflood[floodcell + 6]; 

      //East
      if (!(CHECKBIT(wallinfo, BIT1) ) )   
         neighbour_val[1] = mazeflood[floodcell + 1]; 

      //South
      if (!(CHECKBIT(wallinfo, BIT2) ) )  
         neighbour_val[2] = mazeflood[floodcell - 6];

      //West
      if (!(CHECKBIT(wallinfo, BIT3) ) )   
         neighbour_val[3] = mazeflood[floodcell - 1]; 


      x = 0;
      while (x < 3)
      {
         if (neighbour_val[0] > (neighbour_val[x + 1]))
            neighbour_val[0] = neighbour_val[x + 1];

         x+=1;
      }      
      if ((!(floodcell == destination )) && (!(mazeflood[floodcell] == (1+neighbour_val[0]))))

      {
         mazeflood[floodcell] = 1 + neighbour_val[0];         

         //North
         if (!(CHECKBIT(wallinfo, BIT0)))
         {
            stkptr++; 
            stk[stkptr] = floodcell + 6; 
            stk_empty_flag = 0;  
         }


         //East
         if (!(CHECKBIT(wallinfo, BIT1)))
         {
            stkptr++;
            stk[stkptr] = floodcell + 1;
            stk_empty_flag = 0;  
         }


         //South
         if (!(CHECKBIT(wallinfo, BIT2)))
         {
            stkptr++;
            stk[stkptr] = floodcell - 6;
            stk_empty_flag = 0;  
         }


         //West
         if (!(CHECKBIT(wallinfo, BIT3)))
         {
            stkptr++;
            stk[stkptr] = floodcell - 1;
            stk_empty_flag = 0;   
         }

      }                     
   } 
}   //End floodfill

////////////////////////////////////////////////////////////////////////////////////////////// STEP
unsigned char step(void)
{

   unsigned char neighbour_val[] = {255,255,255,255, 1};
   unsigned char wallinfo = 0, x = 0, x2 = 0;

   wallinfo = walldata[current_cell];



   if (!(CHECKBIT(wallinfo, BIT0)))  
      neighbour_val[0] = mazeflood[current_cell + 6];


   if (!(CHECKBIT(wallinfo, BIT1)))  
      neighbour_val[1] = mazeflood[current_cell + 1];


   if (!(CHECKBIT(wallinfo, BIT2)))  
      neighbour_val[2] = mazeflood[current_cell - 6];


   if (!(CHECKBIT(wallinfo, BIT3))) 
      neighbour_val[3] = mazeflood[current_cell - 1];


   x = 0;
   x2 = 0;
   while (x < 3)
   {

      if (neighbour_val[0] > neighbour_val[x + 1])
      {
         x2 = x + 1;
         neighbour_val[4] = pow(2, ( 2 * x2 )) ; 
         neighbour_val[0] = neighbour_val[x + 1];

      }
      x += 1;
   }  

   if (current_dir == neighbour_val[4]) 
   {
      neighbour_val[4] = 'M';
      return neighbour_val[4];
   }

   if ( ((current_dir == 64) && (neighbour_val[4] == 1))
      || (current_dir == (neighbour_val[4]/4)) )
   {
      neighbour_val[4] = 'R';
      return neighbour_val[4];
   }

   if ( ((current_dir == 1) && (neighbour_val[4] == 64))
      || (current_dir == (4 * neighbour_val[4])) )
   {
      neighbour_val[4] = 'L';
      return neighbour_val[4];
   }

   neighbour_val[4] = 'F';
   return neighbour_val[4];

} 

void offset()
{
      while (error_front_l>30||error_front_r>30)
      {
          
               if (error_front_l>30) // lech phai => re trai
               {
                  dir_l=0;
                  dir_r=1;
                  step_l=0;
                  step_r=0;
                  delay_ms(1);
                  step_l=1;
                  step_r=1;
               }
               if (error_front_r>30) // lech phai => re phai
               {
                  dir_l=1;
                  dir_r=0;
                  step_l=0;
                  step_r=0;
                  delay_ms(1);
                  step_l=1;
                  step_r=1;
               }
      }
               // kiem tra khoang cach
          if (front_l>100||front_r>100)
          {
               while ((front_l>340&&front_r>340)) 
               {
                  dir_l=0;
                  dir_r=0;
                  step_l=0;
                  step_r=0;
                  delay_us(800);
                  step_l=1;
                  step_r=1;
               }
               while ((front_l<300&&front_r<300)) 
               {
                  dir_l=1;
                  dir_r=1;
                  step_l=0;
                  step_r=0;
                  delay_us(800);
                  step_l=1;
                  step_r=1;
               }
          }              
}
void turnleft()
{
int16 i=0;
   dir_l=0;
   dir_r=1;
   for (i=0;i<270;i++)
   {
      step_l=0;
      step_r=0;
      delay_us(600);
      step_l=1;
      step_r=1;
      delay_us(2);
   }
      offset();
}
void turnright()
{
int16 i=0;
   dir_l=1;
   dir_r=0;
   for (i=0;i<270;i++)
   {
      step_l=0;
      step_r=0;
      delay_us(600);
      step_l=1;
      step_r=1;
      delay_us(2);
   }
      offset();
}
void movestraight()
{
int16 i=0;
unsigned char a=0;
   for (i=0;i<782;i++)
   {
         // offset
     if (speed<2)
     {  
      // lech trai => turn right
         if (left>290) 
         {
            dir_l=1;
            dir_r=0;
            step_l=0;
            step_r=0;
            delay_us(500);
            step_l=1;
            step_r=1;
         }
         // lech phai => re trai
         if (right>290)
         {
            dir_l=0;
            dir_r=1;
            step_l=0;
            step_r=0;
            delay_us(500);
            step_l=1;
            step_r=1;
         }
         else a=1;
     }
     else if (speed>=2)
     {        
      // lech trai => turn right
         if (left>320) 
         {
            dir_l=1;
            dir_r=0;
            step_l=0;
            step_r=0;
            delay_us(350);
            step_l=1;
            step_r=1;
         }
         // lech phai => re trai
         if (right>320)
         {
            dir_l=0;
            dir_r=1;
            step_l=0;
            step_r=0;
            delay_us(350);
            step_l=1;
            step_r=1;
         }
         else a=1;
     }
 // ket thuc offset
 // => di thang
            if (a==1)
            {
            dir_l=1;
            dir_r=1;
            step_l=0;
            step_r=0;
            delay_us(delay);
            step_l=1;
            step_r=1;
            a=0;
            }
            // 
            if (front_r>=340||front_l>=340) 
            {
               i=1551;
            }            
   }      
   offset();
   if (speed<2)
   {
      if (left>50&&right<50&&front_l>50&&front_r>50&&(left>300||left<200))
      {
         turnleft();
         turnright();
      }
      else if (right>50&&left>50&&front_l>50&&front_r>50&&(right>300||right<200))
      {
         turnright();
         turnleft();
      }
      offset();
   }
}
///////////////// speed RUN
void speed_run()
{
   unsigned char i;     
      for (i=0;i<3;i++)
      {
         speaker=1;
         delay_ms(40);
         speaker=0;
         delay_ms(40);
      }
   unsigned char mazeflood[36]={ 
   6, 5, 4, 3, 4, 5,
   5, 4, 3, 2, 3, 4,
   4, 3, 2, 1, 2, 3,
   3, 2, 1, 0, 1, 2,
   4, 3, 2, 1, 2, 3,
   5, 4, 3, 2, 3, 4
   };  
   destination = 21;
   speed++;
      if (speed>=2)
      {
         timer0set = 65490;
         delay=280;         
      }
}
