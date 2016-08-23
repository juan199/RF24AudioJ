
/* RFAudio Library: by TMRh20 2011-2014*/

#ifndef __RF24AudioJ_H__
#define __RF24AudioJ_H__

/**
 * @file RF24Audio.h
 *
 * Class declaration for RF24Audio Library
 */

class RF24;

/**
 * Zebra193 2016 - RF24AudioJ: Arduino Realtime Audio Streaming library for Guitar/Bass (with effects)
 *
 * This class implements an Audio Streaming library using nRF24L01(+) radios driven
 * by the Optimized RF24 library (https://github.com/TMRh20/RF24) and the RF24Audio library (http://tmrh20.github.io/)
 * 
 */

class RF24AudioJ
{

public:
  /**
   * Setup the radio and radio identifier
   * @note Changing radioNum is only required if utilizing private node-to-node communication as
   * opposed to broadcasting to the entire radio group
   *
   * @code
   *	RF24 radio(48,49);				// Initialize the radio driver
   *	RF24AudioJ rfAudioJ(radio,0);		// Initialize the audio driver
   * @endcode
   *
   * @param _radio   The underlying radio driver instance
   * @param radioNum The radio identifier
   */
	RF24AudioJ(RF24& _radio, byte radioNum);

  /**
   * Initialize the radio and audio library
   *
   * Generally called in setup to initialize the radio as a Rx
   * @code
   *	rfAudioJ.begin();
   * @endcode
   */
	void begin();

    /**
   * Initialize the radio and audio library
   *
   * Generally called in setup to initialize the radio as a Tx
   * @code
   *	rfAudioJ.begin_Tx();
   * @endcode
   */
	void begin_Tx();

  /**
   * Control transmission through code
   * @code
   *	rfAudioJ.transmit(); // Begin realtime audio streaming
   * @endcode
   * Call this function to begin transmission
   *
   */
	void transmit();

  /**
   * Stop transmission through code
   * @code
   *	rfAudioJ.receive(); // Stop audio streaming
   * @endcode
   * Call this function to stop transmission
   *
   */
	void receive();

  /**
   * Control of Private or Public Communication
   *
   * Call this function to establish private communication between nodes
   * in a radio group, or to switch back to public transmission.
   * @note Using a radioID of 255 will disable private communication and broadcast to all nodes
   *
   * @code
   *	rfAudioJ.broadcast(1); 	// Only transmit audio to radio number 1
   *	rfAudioJ.broadcast(255);  // Transmit audio to all radios in the group
   * @endcode
   * @param radioID  Set the radioID of the radio to communicate privately with.
   */
	void broadcast(byte radioID);

   /**
   * Get any of the preset radio addresses
   *
   * Useful for listening nodes who wish to create private or additional radio groups
   * The library has 14 predefined radio addreses. All radios listen/write on the first
   * two addresses (0,1), and engage a private channel based on the radio number.
   * Radio 0 listens on address 2, Radio 1 on address 3, etc.
   *
   * @code
   *	uint64_t newAddress = rfAudioJ.getAddress(3);	// Gets the 3rd defined radio address
   *    OR
   *	radio.openReadingPipe(0,rfAudioJ.getAddress(7)); // Listens on the 7th defined radio address
   * @endcode
   * @param addressNo  Numbers 0 through 14 to access any part of the defined address array
   * @return RadioAddress:  Returns the requested predefined radio address
   */
    uint64_t getAddress(byte addressNo);


private:
	RF24& radio;
	void timerStart();
};

void TX();
void RX();

#endif



