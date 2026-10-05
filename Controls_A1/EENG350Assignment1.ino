const int A_PIN = 2;
const int B_PIN = 4;

// Changed inside the interrupt
volatile long encoderCount = 0;

// Lets loop() know the encoder moved
volatile bool encoderChanged = false;


// Runs whenever channel A changes
void encoderISR()
{
    int A = digitalRead(A_PIN);
    int B = digitalRead(B_PIN);

    // Determine direction
    if (A == B)
    {
        encoderCount += 2;
    }
    else
    {
        encoderCount -= 2;
    }

    encoderChanged = true;
}


void setup()
{
    Serial.begin(9600);

    pinMode(A_PIN, INPUT_PULLUP);
    pinMode(B_PIN, INPUT_PULLUP);

    // Run encoderISR on both rising and falling edges
    attachInterrupt(
        digitalPinToInterrupt(A_PIN),
        encoderISR,
        CHANGE
    );

    Serial.println("Encoder with interrupts");
}


void loop()
{
    // Print the count after the encoder moves
    if (encoderChanged)
    {
        long countCopy;

        // Copy the shared value without allowing the ISR
        // to change it during the copy
        noInterrupts();

        countCopy = encoderCount;
        encoderChanged = false;

        interrupts();

        Serial.print("Interrupt Count = ");
        Serial.println(countCopy);
    }
}
