// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"
#include <stdlib.h>

typedef int boolean;
#define true 1;
#define false 0;

#define INPUTFILE "test.pgm"
#define OUTPUTFILE "TestOut.pgm"

#define ITERATIONSIZE 100

#define  IMHT 16                //image height
#define  IMWD 16                //image width

#define ClearBit(A,k)   ( A[(k/32)] &= ~(1 << (k%32)) )
#define SetBit(A,k)     ( A[(k/32)] |= (1 << (k%32)) )
#define TestBit(A,k)    ( A[(k/32)] & (1 << (k%32)) )

//#define TestBitValue(val,bit) (if(bit) { val = 1; } else val = 0; )


#define giveModulo(k) ((k+IMHT)%IMHT)

typedef unsigned char uchar;      //using uchar as shorthand

port p_scl = XS1_PORT_1E;         //interface ports to orientation
port p_sda = XS1_PORT_1F;

#define FXOS8700EQ_I2C_ADDR 0x1E  //register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////

void DataInStream(char infname[], chanend c_out)
{
  int res;
  uchar line[ IMWD ];
  printf( "DataInStream: Start...\n" );

  //Open PGM file
  res = _openinpgm( infname, IMWD, IMHT );
  if( res ) {
    printf( "DataInStream: Error openening %s\n.", infname );
    return;
  }

  //Read image line-by-line and send byte by byte to channel c_out
 // int counterTemp = 0;
  for( int y = 0; y < IMHT; y++ ) {
    _readinline( line, IMWD );
  // printf("%d \t",y);
    for( int x = 0; x < IMWD; x++ ) {
      c_out <: line[ x ];
     // printf( "-%4.1d ", line[ x ] ); //show image values
    }
   // printf( "\n" );
  }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  return;
}

int giveModuloFunc(int num)
{
    return ((num+IMHT)%IMHT);
}


int indexOf(int row, int col)
{
    return row*(IMWD)+col;
}

int applyRules(int counter,int currentCellValue)
{

    int result=0;
    if(counter < 2) // less than 2 means death
    {
        result = 0;
    }
    if((currentCellValue != 0)&&((counter == 2)||(counter==3))) // If alive & 2 or 3 neighbours then live
    {
        result = 255;
    }  // Live
    if(counter > 3) // Too many people, death
    {
        result = 0;
    }
    if(currentCellValue == 0 && counter == 3) // Death+ 3 neighbours = Alive
    {
        result = 255;
    }
   // printf("#####Applying Rules for [[%d]] returning {%d} \n",counter,result);
    return result;

}



void workerFunction(chanend dis_comm, chanend above_worker, chanend below_worker,int workerID)
{
        int bit_work_map[(((IMHT/4)+2)*(IMWD))/32];
        int new_bit_map[(((IMHT/4)+2)*(IMWD))/32];


        int height = IMHT/4 +2;
        int width = IMWD;
        // Size of bit_map would be whole width * 1/4 of the height + buffer


        int inValue;
        for(int i=0;i<height;i++)  // Getting input from distributor
        {
            for(int j=0;j<width;j++)
            {
                dis_comm :> inValue;
                if(inValue == 0) { ClearBit(bit_work_map,indexOf(i,j)); } else (SetBit(bit_work_map,indexOf(i,j)));
            }
        }

        boolean workOn = true;
        int iterationNum = 1;
      while(workOn == 1) // Work on the maps (input from Distributor)
      {

                if(iterationNum > 1) // Loop updates the buffer rows.
                {

                    int copy_bottom[IMWD];
                    int copy_top[IMWD];

                    int testingValue;


                    for(int i=0;i<height;i++)
                    {
                        for(int j=0;j<IMWD;j++) // Updates the work map to the last iteration result.
                        {
                            if(TestBit(new_bit_map,indexOf(i,j))) { SetBit(bit_work_map,indexOf(i,j)); } else ClearBit(bit_work_map,indexOf(i,j));
                        }
                    }

                    if(workerID == 1 || workerID == 3) // These two send
                    {
                        for(int i=0;i<IMWD;i++)
                        {
                            if(TestBit(new_bit_map,indexOf(1,i))) { above_worker <: 1; } else above_worker <: 0;
                            if(TestBit(new_bit_map,indexOf(height-2,i))) { below_worker <: 1;} else below_worker <: 0;
                        }
                    }

                    int below_val;
                    int above_val;
                    if(workerID == 2 || workerID == 4) // These two receive at the same time
                    {
                        for(int i=0;i<IMWD;i++)
                        {
                            below_worker :> below_val; //work_map[height-1][i];
                            above_worker :> above_val; //work_map[0][i];
                            if(below_val == 0) { ClearBit(bit_work_map,indexOf(height-1,i)); } else SetBit(bit_work_map,indexOf(height-1,i));
                            if(above_val == 0) { ClearBit(bit_work_map,indexOf(0,i)); } else SetBit(bit_work_map,indexOf(0,i));
                        }
                    }


                    if(workerID == 1 || workerID == 3) // these two now receive while the others send
                    {
                        for(int i=0;i<IMWD;i++)
                        {

                            below_worker :> below_val;  //work_map[height-1][i];
                            above_worker :> above_val; //work_map[0][i];
                            if(below_val == 0) { ClearBit(bit_work_map,indexOf(height-1,i)); } else SetBit(bit_work_map,indexOf(height-1,i));
                            if(above_val == 0) { ClearBit(bit_work_map,indexOf(0,i)); } else SetBit(bit_work_map,indexOf(0,i));

                        }
                    }
                    if(workerID == 2 || workerID == 4)
                    {
                        for(int i=0;i<IMWD;i++)
                        {
                            if(TestBit(new_bit_map,indexOf(1,i))) { above_worker <: 1; } else above_worker <: 0;
                            if(TestBit(new_bit_map,indexOf(height-2,i))) { below_worker <: 1;} else below_worker <: 0;
                        }
                    }

               } // Done updating the buffer zones



        for(int x=1;x<height-1;x++) // Working through the map looking for neighbours
        {
            for(int y=0;y<width;y++)
            {
                int neighbour[8];

                if(TestBit(bit_work_map,indexOf((x+1),giveModulo(y)))) { neighbour[0] = 1;} else neighbour[0] = 0; // down
                if(TestBit(bit_work_map,indexOf((x-1),giveModulo(y)))) { neighbour[1] = 1;} else neighbour[1] = 0; // up
                if(TestBit(bit_work_map,indexOf((x-1),giveModulo(y+1)))) { neighbour[2] = 1; } else neighbour[2] = 0; // top right
                if(TestBit(bit_work_map,indexOf((x-1),giveModulo(y-1)))) { neighbour[3] = 1; } else neighbour[3] = 0; // top left
                if(TestBit(bit_work_map,indexOf((x+1),giveModulo(y-1)))) { neighbour[4] = 1; } else neighbour[4] = 0; // bottom left
                if(TestBit(bit_work_map,indexOf((x+1),giveModulo(y+1)))) { neighbour[5] = 1; } else neighbour[5] = 0; // bottom right
                if(TestBit(bit_work_map,indexOf((x),giveModulo(y+1)))) { neighbour[6] = 1; } else neighbour[6] = 0; // right
                if(TestBit(bit_work_map,indexOf((x),giveModulo(y-1)))) { neighbour[7] = 1; } else neighbour[7] = 0; // left

                int counter = 0;
                for(int i=0;i<8;i++)
                {
                  if(neighbour[i] != 0)
                    {
                         //if(workerID == 3) { printf("Found neighours \n"); }
                        ++counter;
                        neighbour[i] = 0;
                    }
                }
                int resultValue;
                if(TestBit(bit_work_map,indexOf(x,y))) { resultValue = 1;} else resultValue = 0;
                int result = applyRules(counter,resultValue); // Applies the rules of the game
                if(result == 0) { ClearBit(new_bit_map,indexOf(x,y)); } else SetBit(new_bit_map,indexOf(x,y));
              }
        }

         ++iterationNum;
        dis_comm :> workOn;
      } // End of the while(workOn == 1) loop i.e. the work on the maps has to stop and output has to be done

      for(int i=1;i<height-1;i++) // Sending the values of the worked on map (not buffer zones) to the workers.
      {
          for(int j=0;j<IMWD;j++)
          {
              if(TestBit(new_bit_map,indexOf(i,j))) { dis_comm <: 255; } else dis_comm <: 0; // 255 = alive.
          }
      }

}






//////////////////////////////////////////////
/// * >Takes in all the parts of the array
/// * >Looks through each part for the neighbours
/// * >Changes and returns to the distributor once it's done.
/////////////////////////////////////////////
/* void seqWork(chanend dis_Com)
{
   // uchar work_map[IMHT][IMWD];
   // uchar new_map_array[IMHT][IMWD];

    int bit_work_map[(IMHT*IMWD)/32];
    int new_bit_map[(IMHT*IMWD)/32];
    // Input from Distributor
    for(int i=0;i<IMHT;i++)
    {
        for(int j=0;j<IMWD;j++)
        {
            uchar valueSeq;
            dis_Com :> valueSeq;
            if(valueSeq == 0) { ClearBit(bit_work_map,indexOf(i,j)); }
            else {SetBit(bit_work_map,indexOf(i,j));}

        }
    }

    // Goes through list looking at neighbours
    for(int x=0;x<IMHT;x++)
    {
        //printf("Working %d\n",x);
        for(int y=0;y<IMWD;y++)
        {
            int neighbour[8];
            if(TestBit(bit_work_map,indexOf(giveModulo(x+1),giveModulo(y)))) { neighbour[0] = 1;} else neighbour[0] = 0; // down
            if(TestBit(bit_work_map,indexOf(giveModulo(x-1),giveModulo(y)))) { neighbour[1] = 1;} else neighbour[1] = 0; // up
            if(TestBit(bit_work_map,indexOf(giveModulo(x-1),giveModulo(y+1)))) { neighbour[2] = 1; } else neighbour[2] = 0; // top right
            if(TestBit(bit_work_map,indexOf(giveModulo(x-1),giveModulo(y-1)))) { neighbour[3] = 1; } else neighbour[3] = 0; // top left
            if(TestBit(bit_work_map,indexOf(giveModulo(x+1),giveModulo(y-1)))) { neighbour[4] = 1; } else neighbour[4] = 0; // bottom left
            if(TestBit(bit_work_map,indexOf(giveModulo(x+1),giveModulo(y+1)))) { neighbour[5] = 1; } else neighbour[5] = 0; // bottom right
            if(TestBit(bit_work_map,indexOf(giveModulo(x),giveModulo(y+1)))) { neighbour[6] = 1; } else neighbour[6] = 0; // right
            if(TestBit(bit_work_map,indexOf(giveModulo(x),giveModulo(y-1)))) { neighbour[7] = 1; } else neighbour[7] = 0; // left
            int counter = 0;

            printf("\t For %d,%d \n",x+1,y+1);
              printf("\t%d-%d-%d",neighbour[3],neighbour[1],neighbour[2]);
              printf("\n \t%d-%d-%d",neighbour[7],((int)(work_map[x][y])),neighbour[6]);
              printf("\n \t%d-%d-%d \n",neighbour[4],neighbour[0],neighbour[5]);

/*
            for(int i=0;i<8;i++)
            {
               // printf("Neighbour[%d] : %d \n",i,neighbour[i]);
                if(neighbour[i] != 0)
                {
                    //printf("\t neighbour[%d] = %d",i,neighbour[i]);
                    ++counter;
                    neighbour[i] = 0;
                }
            }

           // printf("NEIGHBOUR 0 == %d##\n ",neighbour[0]);
                int resultValue;
                if(TestBit(bit_work_map,indexOf(x,y))) { resultValue = 1;} else resultValue = 0;
            int result = applyRules(counter,resultValue);
          //  printf("\t %d,%d : Result : %d \n",x+1,y+1,result );

           //new_map_array[x][y] = (uchar) result;
           if(result == 0)
           {
               ClearBit(new_bit_map,indexOf(x,y));
           }
           else SetBit(new_bit_map,indexOf(x,y));
        }
      }
    ///// End of map array iteration

    /////// Sending it back to the distributor

    for(int i=0;i<IMHT;i++)
    {
        for(int j=0;j<IMWD;j++)
        {   int returnTestVal;
            if(TestBit(new_bit_map,indexOf(i,j))) { returnTestVal  = 1;} else returnTestVal = 0;
            dis_Com <: returnTestVal;

        }
    }

}
*/



void toAllWorkers(int value, chanend to_Workers[4])
{
    to_Workers[0] <: value;
    to_Workers[1] <: value;
    to_Workers[2] <: value;
    to_Workers[3] <: value;

}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Start your implementation by changing this function to implement the game of life
// by farming out parts of the image to worker threads who implement it...
// Currently the function just inverts the image
//
/////////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend seq_Com,chanend to_Workers[n],unsigned int n)
{
  uchar val;

  int bit_work_map[((IMHT)*(IMWD))/32];
  int new_bit_map[((IMHT)*(IMWD))/32];

 // timer t;


  //Starting up and wait for tilting of the xCore-200 Explorer
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for Board Tilt...\n" );
  fromAcc :> int value;
  printf( "Processing...\n" );
  printf(" and the limit is : %d\n",((IMHT*IMWD)/32));
  for( int y = 0; y < IMHT; y++)
  {
    for( int x = 0; x < IMWD; x++)
    {
        //int r = rand() %20;
        //val= (r%2==0)?0:255;
        c_in :> val;
        if(val == 0) { ClearBit(bit_work_map,indexOf(y,x)); }
        else SetBit(bit_work_map,indexOf(y,x));
    }
  }

  printf("Added data into the value \n");


// Sending data to worker A ///////////////////////////
  for(int i=IMHT-1;i<IMHT;i++) // the last row
  {
      for(int j=0;j<IMWD;j++)
      {
          if(TestBit(bit_work_map,indexOf(i,j))) { to_Workers[0] <: 1; } else to_Workers[0] <: 0;

      }
  }
  for(int i=0;i<(IMHT/4)+1;i++) // From 0 to 1/4th + 1
  {
      for(int j=0;j<IMWD;j++)
      {
          if(TestBit(bit_work_map,indexOf(i,j))) { to_Workers[0] <: 1; } else to_Workers[0] <: 0;
      }
  }



  printf("Starting timer \n");
  //unsigned long long int timer_result;
  //t :> timer_result;

  // Sending data to Worker 2
  for(int i=IMHT/4-1;i<(IMHT/2)+1;i++) // From 1/4th -1 to half + 1
  {
      for(int j=0;j<IMWD;j++)
      {
          if(TestBit(bit_work_map,indexOf(i,j))) { to_Workers[1] <: 1; } else to_Workers[1] <: 0;
      }
  }
  // Sending data to worker 3
  for(int i=(IMHT/2)-1;i<((IMHT*3)/4)+1;i++) // From Half -1 to 3/4th
  {
      for(int j=0;j<IMWD;j++)
      {
          if(TestBit(bit_work_map,indexOf(i,j))) { to_Workers[2] <: 1; } else to_Workers[2] <: 0;
      }
  }

  // Sending data to Worker 4
  for(int i=((IMHT/4)*(3))-1;i<IMHT;i++) // 3/4th - 1 to max
  {
      for(int j=0;j<IMWD;j++)
      {
          if(TestBit(bit_work_map,indexOf(i,j))) { to_Workers[3] <: 1; } else to_Workers[3] <: 0;
      }
  }
  for(int i=0;i<1;i++) // Sending first row.
  {
      for(int j=0;j<IMWD;j++)
      {
          if(TestBit(bit_work_map,indexOf(i,j))) { to_Workers[3] <: 1; } else to_Workers[3] <: 0;
      }
  }
////////////////////////////////////////////////////////////

          printf(" \n Sent data to the workers \n");

          // Counting number of iterations
          boolean work = true;
          int counterIteration = 1;
          unsigned long long int totalTime;
          unsigned long long int timer_mid_in;
          unsigned long long int timer_mid_out;
          int overflow = 0;
          while(counterIteration < ITERATIONSIZE) // Number of times this runs.
          {
              timer t_mid;
              t_mid:> timer_mid_in;
              toAllWorkers(work,to_Workers); // function returns only when all the workers have finished their work and are ready to recieve the next info from the distributor
              t_mid :> timer_mid_out;

              if(totalTime+((timer_mid_out/(10^8)) - (timer_mid_in/(10^8)))>=2147483640)
              {
                  overflow++;
              }
              totalTime = totalTime+((timer_mid_out/(10^8)) - (timer_mid_in/(10^8)));

              //printf("total time at iteration %d is : %d \n",counterIteration,totalTime);
              //printf("--------------------ITERATION %d -----------\n",counterIteration);
              counterIteration++;
              if(counterIteration >= ITERATIONSIZE) // Once the number of iterations has reached max
              {
                  work = false;
                  toAllWorkers(work,to_Workers);
              }
          }


        int counterRecieve = 0;
        int workerNumber = 0;
        for(int i=0;i<IMHT;i++) // to get input from the workers.
        {
          for(int j=0;j<IMWD;j++)
          {
              if(counterRecieve >= IMHT/4)
              {
                  ++workerNumber;
                  counterRecieve = 0;
              }
              int workerValue;
              to_Workers[workerNumber] :> workerValue;//new_map_array[i][j];
              if(workerValue == 0){ ClearBit(new_bit_map,indexOf(i,j)); } else SetBit(new_bit_map,indexOf(i,j));
              }
          ++counterRecieve;
        }



     //unsigned long long int timer_result_out;

    // t :> timer_result_out;
     printf("Stopping timer \n");
    // printf("%d Timer start  \n",timer_result);
     //printf("%d timer end  \n ",timer_result_out);

    // unsigned long long int timer_result_final = timer_result_out - timer_result;
    // printf(" TOTAL TIME TAKEN :%d  \n",timer_result_final);
     printf(" Total Time Taken over %d iterations is : %d \n",ITERATIONSIZE,totalTime);
     printf("with %d overflows in int of size \"2,147,483,647\" \n",overflow);

       for(int x=0;x<IMHT;x++)
       {
           for(int y=0;y<IMWD;y++)
           {
               uchar printValue;
               if(TestBit(new_bit_map,indexOf(x,y))) { printValue = 255; } else {printValue = 0; }
               if(counterIteration == ITERATIONSIZE) { c_out <: printValue; }
               //printf("-%d-\t",printValue);
           }
           // printf("\n");
       }

      //printf("\n");




/*///////////////////////////////////////SEQUENTIAL///////////////////////
 //printf("Sent everything to sequential function \n");
 // Receive value from the sequential program.
  for(int x=0;x<IMHT;x++)
  {
      for(int y=0;y<IMWD;y++)
      {
          int testBit;
          seq_Com :> testBit;
          if(testBit == 0){ClearBit(new_bit_array,indexOf(x,y));}
          else SetBit(new_bit_array,indexOf(x,y));
      }
  }

*////////////////////////////////////////////SEQUENTIAL//////////////////

  printf( "\n processing rounds completed...\n" );

}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{
  int res;
  uchar line[ IMWD ];

  //Open PGM file
  printf( "DataOutStream: Start...\n" );
  res = _openoutpgm( outfname, IMWD, IMHT );
  if( res ) {
    printf( "DataOutStream: Error opening %s\n.", outfname );
    return;
  }

  //Compile each line of the image and write the image line-by-line
  for( int y = 0; y < IMHT; y++ ) {
    for( int x = 0; x < IMWD; x++ ) {
      c_in :> line[ x ];
    }
    _writeoutline( line, IMWD );
    //printf( "DataOutStream: Line written...\n" );
  }

  //Close the PGM image
  _closeoutpgm();
  printf( "DataOutStream: Done...\n" );
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and  read orientation, send first tilt event to channel
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist) {
  i2c_regop_res_t result;
  char status_data = 0;
  int tilted = 0;

  toDist <: 1;
/*
  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }
  
  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the orientation x-axis forever
  while (1) {

    //check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);



    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //send signal to distributor after first tilt
    if (!tilted) {
      if (x>30) {
        tilted = 1 - tilted;
        toDist <: 1;
      }
    }
  }
  */
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {



i2c_master_if i2c[1];               //interface to orientation

char infname[] = INPUTFILE;     //put your input image path here
char outfname[] = OUTPUTFILE; //put your output image path here
chan c_inIO, c_outIO, c_control;    //extend your channel definitions here
chan to_Seq;
chan toWorkers[4];
chan workerComms[4];

par {
    i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
    orientation(i2c[0],c_control);        //client thread reading orientation data
    DataInStream(infname, c_inIO);          //thread to read in a PGM image
    DataOutStream(outfname, c_outIO);       //thread to write out a PGM image
    distributor(c_inIO, c_outIO, c_control, to_Seq,toWorkers,4);//thread to coordinate work on image
    workerFunction(toWorkers[0],workerComms[3],workerComms[0],1); // Worker A
    workerFunction(toWorkers[1],workerComms[0],workerComms[1],2); // Worker B
    workerFunction(toWorkers[2],workerComms[1],workerComms[2],3); // Worker C
    workerFunction(toWorkers[3],workerComms[2],workerComms[3],4); // Worker D
    //seqWork(to_Seq);
  }

  return 0;
}
