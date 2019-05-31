/*
 The circuit:
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 * LED attached from pin 13 to ground (or use the built-in LED on
   most Arduino boards)
 */

// this constant won't change:
const byte ledPin = 13;      // the pin that the LED is attached to
const byte interruptPin = 2; // the pin that the pushbutton is attached to


// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

byte led_state = LOW;
volatile int FSM_state = 0;

#define DEBOUNCE_DELAY 100 // in ms
#define CHANGE_DELAY 500 // in ms

volatile uint32_t last_interrupt_time = 0;
uint32_t last_time = 0;
//------------------------------------------------------------------------------
void setup() {
  // initialize the button pin as a input:
  pinMode(interruptPin, INPUT);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink_ISR, RISING);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink, FALLING);
   
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, led_state);
  // initialize serial communication:
  Serial.begin(9600);
}
//------------------------------------------------------------------------------
void loop() {
	switch(FSM_state){
	case 0: 
	//null
	break;
  
	case 1: 
  //detachInterrupt(digitalPinToInterrupt(interruptPin));

  botton_status();
	break;

  case 2: //not used
  FSM_state=3;
  break;
  
	case 3: 
	led_on();
	FSM_state=0;
	break;
  
  //default:
    }
}
//------------------------------------------------------------------------------
void blink_ISR() {
//Debounce Routine  
  uint32_t interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY) {
  FSM_state = 1;
  }
  last_interrupt_time = interrupt_time;

  }
//------------------------------------------------------------------------------
void botton_status(){
   /*Serial.println("Int OFF");
   Serial.println("Status 1");
   Serial.println("Botton Pressed...");
*/  
      // read the pushbutton input pin:
  buttonState = digitalRead(interruptPin);
    if (buttonState == HIGH) {
		 uint32_t time_now = millis();
     /*Serial.print("Time: ");
     Serial.print(time_now);
     Serial.print(" - ");
     Serial.println(last_time);*/
		if (time_now - last_time > CHANGE_DELAY) {	
			Serial.print("Bottone Premuto: ");
			led_state = !led_state;
      last_time=time_now;
			buttonPushCounter++;
			Serial.println(buttonPushCounter);
			FSM_state=3;
		}
    }
    else {
		led_null();
    }
}
//------------------------------------------------------------------------------
void led_on(){
  //Serial.println("Status 3");
  digitalWrite(ledPin, led_state);
  /*Serial.print("Led status: ");
  Serial.println(led_state);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink_ISR, RISING);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink_ISR, CHANGE);
  Serial.println("Int ON");*/
}
//------------------------------------------------------------------------------

void led_null(){
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink_ISR, RISING);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink_ISR, CHANGE);
FSM_state=0;
}
//------------------------------------------------------------------------------
