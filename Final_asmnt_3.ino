//FreeRTOS library
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

//---------------------------------------------------------------------------------------------------------------

#define DEBOUNCE_TIME_MS 50 // Debounce task delay
int LED_PIN = 4; //output port for blink LED on debounce button event
int BUTTON_PIN = 2; //input port for debounce button
int task_1_led = 25; //output port for LED of task 1 
int task_2_frq = 12;//input port to measure task-2 frequency
int task_3_frq = 14;//input port to measure task-3 frequency
int task4_ptn = 26;//input port to show analog frequency
int task_4_led_err = 27;//output port to blink the led for error using potentiometer
float frequency_1; // Measure frequency of task2
float frequency_2; // Measure frequency of task3

TaskHandle_t buttonTaskHandle, ledTaskHandle, task1Handle, task2Handle, task3Handle, task4Handle, task5Handle; // Tasks which is executed
QueueHandle_t eventQueue; // event queue

//---------------------------------------------------------------------------------------------------------------

struct ButtonEvent {
  uint32_t timestamp; // Timestamp 
  bool state;
};

//---------------------------------------------------------------------------------------------------------------

void buttonTask(void *pvParameters) {
  ButtonEvent lastEvent = {0, false};
  while (true) {
    bool state = digitalRead(BUTTON_PIN); // Read button state and timestamp
    uint32_t timestamp = millis();

    // Debounce button state
    if (state != lastEvent.state || timestamp - lastEvent.timestamp > DEBOUNCE_TIME_MS) {
      lastEvent = {timestamp, state};

      // Send button event to event queue
      xQueueSendToBack(eventQueue, &lastEvent, portMAX_DELAY);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // task delay execution time
  }
}

//---------------------------------------------------------------------------------------------------------------

void ledTask(void *pvParameters) {
  bool ledState = false;
  while (true) {
    ButtonEvent event; // Wait for button event
    xQueueReceive(eventQueue, &event, portMAX_DELAY); // receive queue data using eventqueue

    // Toggle LED state on button press
    if (event.state) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
}

//---------------------------------------------------------------------------------------------------------------

void task1(void *pvParameters) {
  while (true) {
    Serial.println("Task 1 running");
      digitalWrite(task_1_led, HIGH); // set pin 2 high for 200us
      delayMicroseconds(200);
      digitalWrite(task_1_led, LOW); // set pin 2 low for 50us
      delayMicroseconds(50);
      digitalWrite(task_1_led, HIGH); // set pin 2 high for 30us
      delayMicroseconds(30);
      digitalWrite(task_1_led, LOW); // set pin 2 low for remaining period
    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

//---------------------------------------------------------------------------------------------------------------

void task2(void *pvParameters) {
  while (true) {
    Serial.println("Task 2 running");

    int count = 0;
  count += pulseIn(task_2_frq, HIGH); // measure the pulse width of the input signal
  count = count*2;
  frequency_1 = 1000000.0 / (count); // calculate frequency in Hz
  frequency_1 = constrain(frequency_1, 333, 1000); // frequency between 333 and 1000 Hz
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

//---------------------------------------------------------------------------------------------------------------

void task3(void *pvParameters) {
  while (true) {
    Serial.println("Task 3 running");

    int count2 = 0;
  count2 += pulseIn(task_3_frq, HIGH); // measure the pulse width of the input signal
  count2 = count2*2;
  frequency_2 = 1000000.0 / (count2 ); // calculate frequency in Hz
  frequency_2 = constrain(frequency_2, 500, 1000); // frequency between 500 and 1000 Hz
    
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

//---------------------------------------------------------------------------------------------------------------

void task4(void *pvParameters) {
  while (true) {
    Serial.println("Task 4 running");

    const int max_analog_input = 1023;
  const int num_readings = 4;
  int readings[num_readings];
  int indx = 0;
  int total = 0;
  int fltr_val = 0;
  for (int i = 0; i < num_readings; i++) 
  {
  readings[i] = 0;
  }
  
  int analogValue = analogRead(task4_ptn);   // Read analog input
  total = total - readings[indx]; // Subtract the oldest reading from the total
  total = total + analogValue;   // Add to the total
  readings[indx] = analogValue;  // Store reading in the readings array
  indx++;  // Increment the indx
  
  // Wrap the indx if it exceeds the number of readings
  if (indx >= num_readings)
   {
    indx = 0;
   }
   
  fltr_val = total / num_readings; // Compute the filtered value of 4 readings
  
  // If the filtered value is greater than half of the maximum range, turn the led on esle led off
  
  if (fltr_val > max_analog_input / 2) {
    digitalWrite(task_4_led_err, HIGH);
  } 
  else {
    digitalWrite(task_4_led_err, LOW);
  }
 // Serial.println(fltr_val);
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

//---------------------------------------------------------------------------------------------------------------

void task5(void *pvParameters) {
  while (true) {
    Serial.println("Task 5 running");
  frequency_1 = map(frequency_1, 333, 1000, 0, 99); // map frequency between to 0-99
  frequency_2 = map(frequency_2, 500, 1000, 0, 99); // map frequency value between 0-99
  
  // Send the frequency values to the serial port
  Serial.println(frequency_1); //To print frequency of Task2
  Serial.println(frequency_2); //To print frequency of Task3
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//---------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600); // for serial monitor
  pinMode(LED_PIN, OUTPUT); // output led blink on button debounce event
  pinMode(BUTTON_PIN, INPUT_PULLUP); // input pullup as it debounce button 
  pinMode(task_1_led, OUTPUT); // output task1 led 
  pinMode(task_2_frq, INPUT); // input task2 pin measure frequency
  pinMode(task_3_frq, INPUT); // input task3 pin measure frequency
  pinMode(task4_ptn, INPUT); // input task4 pin for potentiometer
  pinMode(task_4_led_err, OUTPUT); // output task4 led to display error
  
  eventQueue = xQueueCreate(10, sizeof(ButtonEvent)); // event queue created
  
  // Create button, LED and 5 tasks from assignment 2
  xTaskCreate(buttonTask, "Button Task", 1024, NULL, 1, &buttonTaskHandle);
  xTaskCreate(ledTask, "LED Task", 1024, NULL, 1, &ledTaskHandle);
  xTaskCreate(task1, "Task 1", 1024, NULL, 2, &task1Handle);
  xTaskCreate(task2, "Task 2", 1024, NULL, 2, &task2Handle);
  xTaskCreate(task3, "Task 3", 1024, NULL, 2, &task3Handle);
  xTaskCreate(task4, "Task 4", 1024, NULL, 2, &task4Handle);
  xTaskCreate(task5, "Task 5", 1024, NULL, 2, &task5Handle);
}

//---------------------------------------------------------------------------------------------------------------

void loop(){
  // do nothing...!!!
  }
