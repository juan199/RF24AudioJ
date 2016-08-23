#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#include <stddef.h>

#include "RF24AudioJ.h"
#include "RF24.h"
#include <userConfig.h>

/**
 * Zebra193 2016 - RF24AudioJ: Arduino Realtime Audio Streaming library for Guitar/Bass (with effects)
 *
 * This class implements an Audio Streaming library using nRF24L01(+) radios driven
 * by the Optimized RF24 library (https://github.com/TMRh20/RF24) and the RF24Audio library (http://tmrh20.github.io/)
 * 
 */


//******* General Variables ************************

volatile boolean buffEmpty[2] = {true,true}, whichBuff = false, a, lCntr=0, streaming = 0, transmitting = 0;
volatile byte buffCount = 0;
volatile byte pauseCntr = 0;
unsigned int intCount = 0;
byte txCmd[2] = {'r','R'};
byte buffer[2][buffSize+1];
char volMod = -1;
byte bitPos = 0, bytePos = 25;
byte bytH;
byte radioIdentifier;

//===================================================================================
// 			Variables needed to implement the Bothrops_Asper_Effect (distortion)
//===================================================================================
byte Lee_Pot;  // Stores the read value from a pot (between 0V-5V: 0-225)
byte VTHL;     // Lower voltage threshold 
byte VTHH;	   // Upper voltage threshold
//===================================================================================

unsigned long volTime = 0;

RF24 radi(0,0);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || (__AVR_ATmega32U4__) || (__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || (__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
  #define rampMega
#endif

const byte broadcastVal = 255; // The value for broadcasting to all nodes using the broadcast() command.

/*****************************************************************************************************************************/
/************************************************* General Section ***********************************************************/

RF24AudioJ::RF24AudioJ(RF24& _radio, byte radioNum): radio(_radio){
	radi = radio;
	radioIdentifier = radioNum;
}

void RF24AudioJ::begin(){ // Set-up the radio as Rx

  radio.begin();
  //delay(500);
  // Set the defined input pins as inputs with pullups high. See http://arduino.cc/en/Tutorial/InputPullupSerial
 // #if defined (ENABLE_LED)
  	pinMode(ledPin,OUTPUT);
 // #endif
  pinMode(speakerPin,OUTPUT); 		pinMode(speakerPin2,OUTPUT);
  pinMode(TX_PIN,INPUT_PULLUP);		pinMode(VOL_UP_PIN,INPUT_PULLUP);
  pinMode(VOL_DN_PIN,INPUT_PULLUP); pinMode(REMOTE_TX_PIN,INPUT_PULLUP);
  pinMode(REMOTE_RX_PIN,INPUT_PULLUP);

  radio.setChannel(1);                 // Set RF channel to 1
  radio.setAutoAck(0);                 // Disable ACKnowledgement packets
  radio.setDataRate(RF_SPEED);         // Set data rate as specified in user options
  radio.setCRCLength(RF24_CRC_8);
  radio.openWritingPipe(pipes[0]);     // Set up reading and writing pipes. All of the radios write via multicast on the same pipe
  radio.openReadingPipe(1,pipes[1]);   // All of the radios listen by default to the same multicast pipe
  radio.openReadingPipe(2,pipes[radioIdentifier + 2]); // Every radio also has its own private listening pipe

  radio.startListening();              // NEED to start listening after opening a reading pipe
  timerStart();						   // Get the timer running
  RX();								   // Start listening for transmissions

}




void RF24AudioJ::begin_Tx(){ // Set-up the radio as TX

  radio.begin();
  //delay(500);
  // Set the defined input pins as inputs with pullups high. See http://arduino.cc/en/Tutorial/InputPullupSerial
 // #if defined (ENABLE_LED)
  	pinMode(ledPin,OUTPUT);
 // #endif
  pinMode(speakerPin,OUTPUT); 		pinMode(speakerPin2,OUTPUT);
  pinMode(TX_PIN,INPUT_PULLUP);		pinMode(VOL_UP_PIN,INPUT_PULLUP);
  pinMode(VOL_DN_PIN,INPUT_PULLUP); pinMode(REMOTE_TX_PIN,INPUT_PULLUP);
  pinMode(REMOTE_RX_PIN,INPUT_PULLUP);

  radio.setChannel(1);                 // Set RF channel to 1
  radio.setAutoAck(0);                 // Disable ACKnowledgement packets
  radio.setDataRate(RF_SPEED);         // Set data rate as specified in user options
  radio.setCRCLength(RF24_CRC_8);
  radio.openWritingPipe(pipes[0]);     // Set up reading and writing pipes. All of the radios write via multicast on the same pipe
  radio.openReadingPipe(1,pipes[1]);   // All of the radios listen by default to the same multicast pipe
  radio.openReadingPipe(2,pipes[radioIdentifier + 2]); // Every radio also has its own private listening pipe

  radio.startListening();              // NEED to start listening after opening a reading pipe
  timerStart();						   // Get the timer running
  TX();								   // Start listening for transmissions

}


void RF24AudioJ::timerStart(){                           //            #### Juan: Timer para monitorear los botones #######
    ICR1 = (16000000/SAMPLE_RATE);                  //Timer will count up to this value from 0;
    TCCR1A = _BV(COM1A1) | _BV(COM1B0) | _BV(COM1B1);  //Enable the timer port/pin as output
	TCCR1A |= _BV(WGM11);                              //WGM11,12,13 all set to 1 = fast PWM/w ICR TOP
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);      //CS10 = no prescaling
	
	
//================================================================================================================
//				JUAN: timer2 set-up for Transmission-Sending Interrupt
//================================================================================================================
	//OCR2A = 25;
	OCR2B = 20;   // Timer will count from 0 up to 100 (approx each 6.25 us) :Valor de prueba: CAMBIAR!!!
	TCCR2A = /*_BV(COM2A1) |*/ _BV(COM2B1);  // Enable the timer port/pin as output
	TCCR2A |= _BV(WGM21) | _BV(WGM20);                              // WGM21, WGM22, WGM20 all set to 1 = fast PWM/w ICR TOP
	TCCR2B =  /*_BV(WGM22) |*/ _BV(CS20);         // CS20 = no prescaling
//================================================================================================================
}

void rampDown(){                                                     // ###### Juan: para apagar el Speaker ####
            int current = OCR1A;
            if(current > 0){
                for(int i=0; i < ICR1; i++){
	          OCR1B = constrain((current - i),0,ICR1);
	          OCR1A = constrain((current - i),0,ICR1);
	          //OCR2B = constrain((current - i),0,ICR1);
	        //for(int i=0; i<10; i++){ while(TCNT1 < ICR1-50){} }
                delayMicroseconds(100);
	      }
            }

}


void rampUp(byte nextVal){                                              // ###### Juan: Para encender el Speaker ####

	byte tmp = 200;
	unsigned int mod;
	if(volMod > 0){ mod = OCR1A >> volMod; }else{ mod = OCR1A << (volMod*-1); }
	if(tmp > mod){
		for(unsigned int i=0; i<buffSize; i++){ mod = constrain(mod+1,mod, tmp); buffer[0][i] = mod; }
		for(unsigned int i=0; i<buffSize; i++){ mod = constrain(mod+1,mod, tmp); buffer[1][i] = mod; }
	}else{
		for(unsigned int i=0; i<buffSize; i++){ mod = constrain(mod-1,tmp ,mod); buffer[0][i] = mod; }
		for(unsigned int i=0; i<buffSize; i++){ mod = constrain(mod-1,tmp, mod); buffer[1][i] = mod; }
	}
	whichBuff = 0; buffEmpty[0] = 0; buffEmpty[1] = 0; buffCount = 0;

}


void RF24AudioJ::transmit(){
	TX();
}
void RF24AudioJ::receive(){
	RX();
}



uint64_t RF24AudioJ::getAddress(byte addressNo){

	return pipes[addressNo];

}
/*****************************************************************************************************************************/
/**											Reception (RX) Section                     										**/
/*****************************************************************************************************************************/

void handleRadio(){
   //byte bufferJ[buffSize+1] = {0x00};

  if(buffEmpty[!whichBuff] && streaming){           // If in RX mode and a buffer is empty, load it
      if(radi.available() ){
		  boolean n=!whichBuff;						// Grab the changing value of which buffer is not being read before enabling nested interrupts
          TIMSK1 &= ~_BV(ICIE1);					// Disable this interrupt so it is not triggered while still running (this may take a while)
	 	  sei();									// Enable nested interrupts (Other interrupts can interrupt this one)
          radi.read(&buffer[n],32);					// Read the payload from the radio
          buffEmpty[n] = 0;                   		// Indicate that a buffer is now full and ready to play
          pauseCntr = 0;                            // For disabling speaker when no data is being received

          TIMSK1 |= _BV(ICIE1);						// Finished, re-enable the interrupt vector that runs this function
      }else{ pauseCntr++; }                         // No payload available, keep track of how many for disabling the speaker
      if(pauseCntr > 50){							// If we failed to get a payload 250 times, disable the speaker output
		  pauseCntr = 0;							// Reset the failure counter
		  rampDown();								// Ramp down the speaker (prevention of popping sounds)
		  streaming = 0;							// Indicate that streaming is stopped
		  TIMSK1 &= ~(_BV(TOIE1) );					// Disable the TIMER1 overflow vector (playback)
		  #if defined (ENABLE_LED)
		  TCCR0A &= ~_BV(COM0A1);					// Disable the TIMER0 LED visualization
		  #endif
		  TCCR1A &= ~(_BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0)); // Disable speaker output
	  }
  }else

  if(!streaming){                  					// If not actively reading a stream, read commands instead
    if(radi.available() ){							// If a payload is available
		TIMSK1 &= ~_BV(ICIE1);						// Since this is called from an interrupt, disable it
	 	sei();										// Enable global interrupts (nested interrupts) Other interrupts can interrupt this one
        radi.read(&buffer[0],32);					// Read the payload into the buffer #### Juan: Commandos por buffer [0][0] ####
        /*switch(buffer[0][0]){                       // Additional commands can be added here for controlling other things via radio command
          #if !defined (RX_ONLY)
          case 'r': if(buffer[0][1] == 'R' && radioIdentifier < 2){  //Switch to TX mode if we received the remote tx command and this is radio 0 or 1
                      TX();
                    }
                    break;
          #endif*/
          streaming= 1;              		// If not a command, treat as audio data, enable streaming


   				   TCCR1A |= _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0);  //Enable output to speaker pin
   				   rampUp(buffer[0][31]);						// Ramp up the speakers to prevent popping
   				   //buffEmpty[0] = false;			// Set the status of the memory buffers
   				   //buffEmpty[1] = true;
          		   TIMSK1 |= _BV(TOIE1);			// Enable the overflow vector
          		   /*#if defined (ENABLE_LED)
          		   TCCR0A |= _BV(COM0A1);			// Enable the LED visualization output
          		   #endif*/
				  // break;

       // }
        TIMSK1 |= _BV(ICIE1);						// Finished: Re-enable the interrupt that runs this function.
    }
  }
}



void RX(){ 											// Start Receiving

		TIMSK1 &= ~_BV(OCIE1B) | _BV(OCIE1A);		// Disable the transmit interrupts
		ADCSRA = 0; ADCSRB = 0;                		// Disable Analog to Digital Converter (ADC)
        buffEmpty[0] = 1; buffEmpty[1] = 1;    		// Set the buffers to empty
        /*#if defined (oversampling)					// If running the timer at double the sample rate
          ICR1 = 10 * (800000/SAMPLE_RATE);    		// Set timer top for 2X oversampling
        #else*/
        // ICR1 = 10 * (1600000/SAMPLE_RATE);		// Timer running at normal sample rate speed
        //#endif

        radi.openWritingPipe(pipes[0]);       		// Set up reading and writing pipes
        radi.openReadingPipe(1,pipes[1]);
        radi.startListening();                		// Exit sending mode
        TIMSK1 = _BV(ICIE1);                  		// Enable the capture interrupt vector (handles buffering and starting of playback)



}

boolean nn = 0;
volatile byte bufCtr = 0;
volatile unsigned int visCtr = 0;

ISR(TIMER1_CAPT_vect){							   // This interrupt checks for data at 1/16th the sample rate. Since there are 32 bytes per payload, it gets two chances for every payload

bufCtr++; visCtr++;								   // Keep track of how many times the interrupt has been triggered

if(bufCtr >= 16){								   // Every 16 times, do this
	handleRadio();							   	   // Check for incoming radio data if not transmitting

	bufCtr = 0;									   // Reset this counter

	if(visCtr >= 32 && streaming){			   	   // Run the visualisation at a much slower speed so we can see the changes better
		OCR0A = buffer[whichBuff][0] << 2;		   // Adjust the TIMER0 compare match to change the PWM duty and thus the brightess of the LED
		visCtr = 0;								   // Reset the visualization counter
	}

}

}


// Receiving interrupt
ISR(TIMER1_OVF_vect){                                      				// This interrupt vector loads received audio samples into the timer register


  if(buffEmpty[whichBuff] ){ whichBuff=!whichBuff; }else{  				// Return if both buffers are empty

    /*#if defined (oversampling)											// If running the timers at 2X speed, only load a new sample every 2nd time
      if(lCntr){lCntr = !lCntr;return;}   lCntr=!lCntr;
    #endif*/


/*************** Standard 8-Bit Audio Playback ********************/
  	  if(volMod < 0 ){                                         			 //Load an audio sample into the timer compare register
		  //OCR1B = 0;
  		  OCR1A = (buffer[whichBuff][intCount] /*>> volMod*-1*/);    //Output to speaker at a set volume 
  		  OCR2B = (buffer[whichBuff][intCount] /*>> volMod*-1*/);
  		  //OCR1A = OCR1B = (buffer[whichBuff][intCount] >> volMod*-1);    //Output to speaker at a set volume 
  	  }else{
		  //OCR1B = 0;
		  OCR1A = buffer[whichBuff][intCount] /*<< volMod*/;
		  OCR2B = buffer[whichBuff][intCount] /*<< volMod*/;
		  //OCR1A = OCR1B = buffer[whichBuff][intCount] << volMod;
  	  }

  	  intCount++;                                              			 //Keep track of how many samples have been loaded

  	  if(intCount >= buffSize){                                			 //If the buffer is empty, do the following
  		  intCount = 0;                                        			 //Reset the sample count
  		  buffEmpty[whichBuff] = true;                        			 //Indicate which buffer is empty
  		  whichBuff = !whichBuff;                              			 //Switch buffers to read from
  	  }

  }
}

/*****************************************************************************************************************************/
/*************************************** Transmission (TX) Section ***********************************************************/
/*****************************************************************************************************************************/
#if !defined (RX_ONLY) //If TX is enabled:

void RF24AudioJ::broadcast(byte radioID){

	if(radioID == radioIdentifier){ return; }						// If trying to send to our own address, return

	noInterrupts();													// Disable interrupts during change of transmission address

	if(radioID == broadcastVal){
			radio.openWritingPipe(pipes[1]);						// Use the public multicast pipe
	}else{
			radio.openWritingPipe(pipes[radioID + 2]);				// Open a pipe to the specified radio(s). If two or more radios share the same ID,
	}																// they will receive the same single broadcasts, but will not be able to initiate
	interrupts();													// private communication betwen each other
}

  //====================================================================================================
  //							Transmission sending interrupt
  //====================================================================================================
	
	 ISR(TIMER1_COMPA_vect){                                      		 // This interrupt vector sends the samples when a buffer is filled
	 if(buffEmpty[!whichBuff] == 0){                      			 // If a buffer is ready to be sent
	 		a = !whichBuff;                      					 // Get the buffer # before allowing nested interrupts
   			TIMSK1 &= ~(_BV(OCIE1A));            					 // Disable this interrupt vector
   			sei();                               					 // Allow other interrupts to interrupt this vector (nested interrupts)
            radi.startFastWrite(&buffer[a],32,0);
            
			//radi.writeFast(&buffer[a],32);						 //  OJO!!!! (ir a RF24.cpp función: startFastWrite )
			
			buffEmpty[a] = 1;                    					 // Mark the buffer as empty
	   	    TIMSK1 |= _BV(OCIE1A);                					 // Re-enable this interrupt vector

     }

  }



  
  //Transmission buffering interrupt [Here's the distortion effect]
  ISR(TIMER1_COMPB_vect){                                            // This interrupt vector captures the ADC values and stores them in a buffer

    //#if !defined (tenBit)                                            // 8-bit samples
	 
	  //===========================================================================================================================================
	  //							Bothrops_Asper_Effect: Hard-Clipping Simple
	  //===========================================================================================================================================
	  // Note: Apply a simple distortion effect clipping the audio signal, use the Tx as effect or transmitter + effect
	  //===========================================================================================================================================
	 Lee_Pot = 160; 	 // Change the value of Lee_pot between 128-255 to vary the level of the threshold,
					     // for low values of Lee_Pot it will has more distortion (but it can affect the volume because the amplitude limited)  
      VTHL = 255-Lee_Pot;
      VTHH = Lee_Pot;
      
      if(ADCH < VTHL && ADCH < 128){
			bytH = VTHL;
			buffer[whichBuff][buffCount] = bytH ;
			}
	  else if(ADCH > VTHH && ADCH > 128 ){
			bytH = VTHH; 
			buffer[whichBuff][buffCount] = bytH ;
            }
      else{
		  bytH = ADCH;
		  buffer[whichBuff][buffCount] = bytH ;
		  }
		  
	  //===========================================================================================================================================
	  //							Bothrops_Asper_Effect: Soft-Clipping Simple
	  //===========================================================================================================================================
	  // Note: Apply a simple distortion effect attenuating the signal over the threshold, use the Tx as effect (transmit is not allowed)
	  //===========================================================================================================================================	  
		  //#define G_MAX 2;
		  
		  /*float G = 1.9;
		  float L = 0.4;
		  bytH = ADCH;
		  byte VT = 100;
		  VTHL = 127 - VT;
		  VTHH = 127 + VT;
		  
		  float f_VTL = float(VTHL);
		  float f_VTH = float(VTHH);
		  float f_Vin = float(bytH); 
		  float f_Vx = f_Vin*G+127*(1-G);
		  unsigned int i_Vx = round(f_Vx);
		  
		  byte Vx = byte(i_Vx);  
		  unsigned int V_sch = round(f_Vx*L + VTHH*(1-L));
		  unsigned int V_scl = round(f_Vx*L + VTHL*(1-L));
		  byte V_fsch = byte(V_sch);
		  byte V_fscl = byte(V_scl);
		  
		  
		  if(f_Vx < VTHL && f_Vx < 127 ){
			 bytH = V_fscl;
			 buffer[whichBuff][buffCount] = bytH;			  
			  }
		  else if(f_Vx > VTHH &&f_Vx > 127 ){
			 bytH = V_fsch;
			 buffer[whichBuff][buffCount] = bytH;
			}
		  else{
			  bytH =Vx;
			  buffer[whichBuff][buffCount] = bytH;
			  }*/
		   
      //=====================================================================================================================================
      
      // Important Note: uncomment the following line to turn-off the effect and just transmit the audio signal
      //buffer[whichBuff][buffCount] = bytH = ADCH;                    // Read the high byte of the ADC register into the buffer for 8-bit samples 

      #if defined (speakerTX)                                        // If output to speaker while transmitting is enabled
        //OCR2B = bytH ;  
		OCR1A = bytH ;     		 // Output to speaker at a set volume if defined
      #endif*/
		
		
    buffCount++;                                         // Keep track of how many samples have been loaded
	if(buffCount >= 32){                                 // In 8-bit mode, do this every 32 samples

    	//Both modes
	  	buffCount = 0;                                   // Reset the sample counter
      	buffEmpty[!whichBuff] = 0;                       // Indicate which bufffer is ready to send
  	  	whichBuff = !whichBuff;                          // Switch buffers and load the other one
  	}
  }


void TX(){
		//TIMSK1 &= ~(_BV(OCIE1A));          				//Disable the Radio Tx
		TIMSK1 &= ~(_BV(ICIE1) | _BV(TOIE1));				 // Disable the receive interrupts
		#if defined (ENABLE_LED)
		TCCR0A &= ~_BV(COM0A1);								 // Disable LED visualization
		#endif
        radi.openWritingPipe(pipes[1]);   				 	 // Set up reading and writing pipes
        radi.openReadingPipe(1,pipes[0]);
        radi.stopListening();							 	 // Enter transmit mode on the radio


        streaming = 0;
  		buffCount = 0; buffEmpty[0] = 1; buffEmpty[1] = 1;   //Set some variables


        byte pin = ANALOG_PIN;
	/*** This section taken from wiring_analog.c to translate between pins and channel numbers ***/
	#if defined(analogPinToChannel)
	#if defined(__AVR_ATmega32U4__)
		if (pin >= 18) pin -= 18; // allow for channel or pin numbers
	#endif
		pin = analogPinToChannel(pin);
	#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		if (pin >= 54) pin -= 54; // allow for channel or pin numbers
	#elif defined(__AVR_ATmega32U4__)
		if (pin >= 18) pin -= 18; // allow for channel or pin numbers
	#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
		if (pin >= 24) pin -= 24; // allow for channel or pin numbers
	#else
		if (pin >= 14) pin -= 14; // allow for channel or pin numbers
	#endif

	#if defined(ADCSRB) && defined(MUX5)
		// the MUX5 bit of ADCSRB selects whether we're reading from channels
		// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
		ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
	#endif

	#if defined(ADMUX)
		ADMUX = (pin & 0x07) | _BV(REFS0); 									//Enable the ADC PIN and set 5v Analog Reference
	#endif

        ICR1 = 10 * (1600000/SAMPLE_RATE);           						//Timer counts from 0 to this value

        #if !defined (speakerTX) 											//If disabling/enabling the speaker, ramp it down
        	rampDown();
            TCCR1A &= ~(_BV(COM1A1));                    					//Disable output to speaker

        #endif
      TCCR1A &= ~(_BV(COM1B1) |_BV(COM1B0) );

      #if !defined (tenBit)
	ADMUX |= _BV(ADLAR);            										//Left-shift result so only high byte needs to be read
      #else
        ADMUX &= ~_BV(ADLAR);           									//Don't left-shift result in 10-bit mode
      #endif

        ADCSRB |= _BV(ADTS0) | _BV(ADTS0) | _BV(ADTS2);           //Attach ADC start to TIMER1 Capture interrupt flag
	byte prescaleByte = 0;

	if(      SAMPLE_RATE < 8900){  prescaleByte = B00000111;} //128
        else if( SAMPLE_RATE < 18000){ prescaleByte = B00000110;} //ADC division factor 64 (16MHz / 64 / 13 clock cycles = 19230 Hz Max Sample Rate )
	else if( SAMPLE_RATE < 27000){ prescaleByte = B00000101;} //32  (38461Hz Max)
	else if( SAMPLE_RATE < 65000){ prescaleByte = B00000100;} //16  (76923Hz Max)
        else   {                       prescaleByte = B00000011;} //8  (fast as hell)

        ADCSRA = prescaleByte;                        //Adjust sampling rate of ADC depending on sample rate
	ADCSRA |= _BV(ADEN) | _BV(ADATE);             //ADC Enable, Auto-trigger enable


       TIMSK1 = _BV(OCIE1B) | _BV(OCIE1A);          //Enable the TIMER1 COMPA and COMPB interrupts
}


#endif
