const byte motor1pinfwd = 5;
const byte motor1pinbwd = 6;
const byte motor2pinfwd = 10;
const byte motor2pinbwd = 11;

void setup()
{
	Serial.begin(9600);
	pinMode(motor1pinfwd, OUTPUT);
	pinMode(motor1pinbwd, OUTPUT);
	pinMode(motor2pinfwd, OUTPUT);
	pinMode(motor2pinbwd, OUTPUT);
}

void loop()
{

	analogWrite(motor1pinfwd, 100);

}
