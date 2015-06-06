byte IRPin = 4;

void setup()
{
	Serial.begin(9600);
	pinMode(IRPin, INPUT);
}

void loop()
{
	Serial.println(digitalRead(IRPin));
}
