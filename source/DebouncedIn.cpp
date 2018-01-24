#include "DebouncedIn.h"
#include "mbed.h"

/*
 * Constructor
 */
DebouncedIn::DebouncedIn(PinName in) 
    : _in(in) {    
        
    // reset all the flags and counters    
    _samples = 0;
    _output = 0;
    _output_last = 0;
    _rising_flag = 0;
    _falling_flag = 0;
    _state_counter = 0;
    
    // Attach ticker
    _ticker.attach(this, &DebouncedIn::_sample, 0.005);     
}
  
void DebouncedIn::_sample() {

    // take a sample
    _samples = _samples >> 1; // shift left
      
    if (_in) {
        _samples |= 0x80;
    }  
      
    // examine the sample window, look for steady state
    if (_samples == 0x00) {
        _output = 0;
    } 
    else if (_samples == 0xFF) {
        _output = 1;
    }


    // Rising edge detection
    if ((_output == 1) && (_output_last == 0)) {
        _rising_flag++;
        _state_counter = 0;
    }

    // Falling edge detection
    else if ((_output == 0) && (_output_last == 1)) {
        _falling_flag++;
        _state_counter = 0;
    }
    
    // steady state
    else {
        _state_counter++;
    }
    
   // update the output
    _output_last = _output;
    
}



// return number of rising edges
int DebouncedIn::rising(void) {
    int return_value = _rising_flag; 
    _rising_flag = 0;
    return(return_value);
}

// return number of falling edges
int DebouncedIn::falling(void) {
    int return_value = _falling_flag; 
    _falling_flag = 0;
    return(return_value);
}

// return number of ticsk we've bene steady for
int DebouncedIn::steady(void) {
return(_state_counter);
}

// return the debounced status
int DebouncedIn::read(void) {
    return(_output);
}

// shorthand for read()
DebouncedIn::operator int() {
    return read();
}


