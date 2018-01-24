#include "mbed.h"

    class DebouncedIn {
        public:      
             DebouncedIn(PinName in);

             int read (void);
             operator int();
              
             int rising(void);
             int falling(void);
             int steady(void);
              
        private :    
               // objects
               DigitalIn _in;    
               Ticker _ticker;

               // function to take a sample, and update flags
               void _sample(void);

               // counters and flags
               int _samples;
               int _output;
               int _output_last;
               int _rising_flag;
               int _falling_flag;
               int _state_counter;

    };
    