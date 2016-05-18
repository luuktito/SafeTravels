#include <NewPing.h>

// ---------------------------------------------------------------------------
// Example Ping library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------


#define TRIGGER_PIN  7 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define PIEZO_PIN 10

NewPing sonar(TRIGGER_PIN, ECHO_PIN, 200); // NewPing setup of pins and maximum distance.

  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
  pinMode(PIEZO_PIN, OUTPUT);
 
void run()
{
  sonar.ping();                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
}