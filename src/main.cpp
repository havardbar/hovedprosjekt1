#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4OLED display;
Zumo32U4Encoders encoder;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Buzzer buzzer;

int mode = 0; // hovedmodus

//
// bateri:
//

// Setter switch case til modus tre som default
int modus = 3;

// millis funksjoner forå få kode uten delay
unsigned long previousMillis_bateri = 0;

// Tall for beregning av lengde
const float Diameter = 3.5;
const int antall_rotasjon = 910; // Tall
const float Pi = 3.14159;
// int lengde_kjort_siden_ladning = 0;
unsigned long tid_kjort = 0; // Tidsfunksjon for å beregne gjennomsnittshastighet.

// Tall for beregning av batterietet
float batteriet = 100;
float lengde_kjort = 0;
const long interval = 2000;

// Startverdi for motoren
long motor_h = 0;
long motor_v = 0;
long motor_v1 = 0;
long motor_h1 = 0;
long forrige_motor_h = 0;
long forrige_motor_v = 0;

// Funksjoner for å kunne behandle negative Encoder verdier.
float CountsDiffV = 0;
float CountsDiffH = 0;
float snitt_hastighet = 0;
float charging_constant = 0;

// Tar for seg registrering av knappen.
int knappe_teller = 0;

//
// linjefølger:
//

// millis for rundeteller
unsigned long currentmillis_round;
unsigned long previusmilllis_round;
const int time_round = 100;

// millis for restart sving
unsigned long currentmillis_restart;
unsigned long previusmilllis_restart;
const int time_restart = 400;

// millis for kjeglefunksjon

unsigned long currentMillis_cone;
unsigned long previusMillis_cone;
const int time_cone = 3000;

int cone = 0;

// array for position
unsigned int lineSensorValues[5];

int16_t position;     // hovedposition fra 0 - 4000
float left_speed;     // fart til venstrebelte
float right_speed;    // fart til høyre belte
int16_t left_speed1;  // verdi av fart med D
int16_t right_speed1; // verdi av fart med D

int16_t lastError;
int16_t error; // lagre hvor langt unna man er streken

int rounds_N; // telle runder

float speed = 400.0;     // standarfart
float speed_1 = speed;   // fart etter snuopperasjon
int16_t speedDifference; // fartsforskjell PID

int on_line = 0;       // hvilken modus, linjefølger
float min_value = 200; // feilmargin verdi, linje modus
int line_value_change = 0;
int first = 0;

//
// knapp modus:
//

// case verdi i firkant modus.
int sq_mode = 0;

// case verdi snu 180 grader
int turning;

// telle verdier
float count_right;
float count_left;
float count_distance;

int reset; // setter en restart verdi

int buttonpress; // case verdi modus

// hindre debaunce
int prevButtonState = LOW;
int buttonState;
int buttonState_B;
int prevButtonstate_B = LOW;

int buttonState_A;
int prevButtonState_A = LOW;

int buttonState_C;
int prevButtonState_C = LOW;

//
// startfunksjoner
//

// startfunksjon til bilen
void sensorStartupp()
{
  display.setLayout11x4(); // skriftstørelse på skjerm
  delay(1000);
  for (int i = 0; i <= 140; i++)
  { // kalibrere sensorene til bilen
    lineSensors.calibrate();
    motors.setSpeeds(-200, 200);
  }
  motors.setSpeeds(0, 0); // sette fart lik 0 så motor ikke blir ødelagt
  delay(150);
}

void setup()
{
  lineSensors.initFiveSensors();
  sensorStartupp();
}

//
// knapp modus;
//

// funksjon som teller hvor langt bilen har kjørt
void distance()
{
  count_left = encoder.getCountsLeft();
  count_right = encoder.getCountsRight();
  count_distance = ((count_left + count_right) / 2) / 7625; // 7625 er ca 1 m, man litt under da
}

// funksjon som restarter tellingen (avstandsmåleren).
void resetdistance()
{
  if (reset == 1)
  {
    motor_h1 = motor_h1 + count_right;
    motor_v1 = motor_v1 + count_left;

    encoder.getCountsAndResetLeft(); // restarter tellefunksjon
    encoder.getCountsAndResetRight();
    count_left = encoder.getCountsLeft(); // henter ut nye telleverdier
    count_right = encoder.getCountsRight();
    count_distance = ((count_left + count_right) / 2) / 7625; // legger inn count distance verdier.
    motors.setSpeeds(0, 0);                                   // stopper moteren

    currentmillis_restart = millis(); // bruker millis istede for delay()
    if (currentmillis_restart - previusmilllis_restart >= time_restart)
    {
      reset = 0; // går ut av funksjonen når det har gått 0.4 sekunder
    }
  }
}

// funskjon for bilen til å kjøre i firkanter
void square()
{

  distance();

  switch (sq_mode)
  {

  case 0:
    resetdistance(); // henter inn resetdistance funksjonen

    if (reset == 0)
    {
      motors.setSpeeds(speed, speed); // setter farten rett frem
    }

    if (count_distance >= 0.3)
    { // når bilen har kjørt en hvis lengde bytter den modus.
      sq_mode = 1;
      reset = 1;
      previusmilllis_restart = millis();
    }
    break;

  case 1:

    resetdistance(); // henter inn resetdistance funksjonen
    if (reset == 0)
    {
      motors.setSpeeds(speed * (-1), speed * (1)); // får bilen til å snu
    }

    if (count_right >= 600)
    { // når bilen har snudd 90 grader byttes modus, når farten er 100 snus den på 600
      sq_mode = 0;
      reset = 1;
      previusmilllis_restart = millis();
    }

    break;
  }
}

// funksjon for sirkel
void circle()
{
  resetdistance();
  motors.setSpeeds(speed * 0.5, speed * 1);
}

void turningopperation()
{
  distance();

  switch (turning)
  {

  case 0:
    resetdistance(); // henter inn resetdistance funksjonen
    if (reset == 0)
    {
      motors.setSpeeds(speed * 0.25, speed * 0.25); // setter farten rett frem
    }

    if (count_distance >= 0.15)
    { // når bilen har kjørt en hvis lengde bytter den modus.
      turning = 1;
      reset = 1;
      previusmilllis_restart = millis();
    }
    break;

  case 1:
    resetdistance(); // henter inn resetdistance funksjonen

    if (reset == 0)
    {
      motors.setSpeeds(speed * (-0.5), speed * (0.5)); // får bilen til å snu
    }

    if (count_right >= 1200)
    { // når bilen har snudd 180 grader byttes modus, når farten er 100 snus den på 1200
      turning = 0;
      reset = 1;
      previusmilllis_restart = millis();
      if (mode == 0)
      {
        speed_1 = speed * 0.4;
      }
    }
    break;
  }
}

void coneDriving()
{
  distance();
  resetdistance(); // henter inn resetdistance funksjonen
  currentMillis_cone = millis();
  if (currentMillis_cone - previusMillis_cone >= time_cone)
  {
    if (reset == 0 and (cone % 2) == 0)
    {
      motors.setSpeeds(speed * 0.3, speed * 0.5);
    }

    if (reset == 0 and (cone % 2) == 1)
    {
      motors.setSpeeds(speed * 0.5, speed * 0.3);
    }

    if (count_distance >= (1 + cone) * 0.3 and cone < 6)
    {
      cone += 1;
    }
    else if (cone >= 6)
    {
      motors.setSpeeds(0, 0);
    }
  }
}

// funksjon for hvilket modus bilen skal være i avhengig av knappetrykk.
void button_mode()
{
  buttonState_A = buttonA.isPressed();
  buttonState_C = buttonC.isPressed();

  if (buttonState_A != prevButtonState_A)
  {
    prevButtonState_A = buttonState_A;
    if (buttonA.isPressed())
    { // kjøre i firkant, knapp A
      buttonpress = 1;
      reset = 1;
      sq_mode = 0;
    }
  }

  if (buttonB.isPressed()) // kjøre i sirkel, knapp B
  {
    buttonpress = 2;
    reset = 1;
  }

  if (buttonState_C != prevButtonState_C)
  {
    prevButtonState_C = buttonState_C;
    if (buttonC.isPressed()) // kjøre frem og tilbake, knapp C
    {
      buttonpress = 3;
      reset = 1;
      turning = 0;
    }
  }

  if ((buttonC.isPressed()) && (buttonA.isPressed()))
  {
    buttonpress = 4;
    reset = 1;
    cone = 0;
    previusMillis_cone = millis();
  }

  switch (buttonpress) // hvilken case avhenger av knappetrykk.
  {
  case 1:
    square();
    break;
  case 2:
    circle();
    break;
  case 3:
    turningopperation();
    break;
  case 4:
    coneDriving();
    break;
  }
}

//
// linje følging:
//

// lese av positisonsverdi
void readValue()
{
  position = lineSensors.readLine(lineSensorValues); // lagrer positisjon
}

void wrongway()
{ // unødvendig funksjon, men lagret slik for å kunne legge inn flere funksjoner senere
  turningopperation();
}

void readSensorvalue()
{
  on_line = 1;
  for (int i = 0; i < 5; i = i + 1)
  {
    if (lineSensorValues[i] >= min_value)
    {              // må ha litt høy min_value pga svarte prikker på gulv, hvis disse ikka hadde hvert der så kunne min_value hver lavere og bilen ville kjørt smudere.
      on_line = 0; // modus linjeføger
    }
  }
  if (lineSensorValues[2] >= 850 and lineSensorValues[4] >= 850)
  {              // modus snu 90 grader
    on_line = 2; // sving høyre etter snu
  }

  if (lineSensorValues[2] >= 800 and lineSensorValues[3] >= 800 and speed_1 == speed) // modus sving venstre istede for sideveier
  {
    on_line = 3; // sving venstre i kryss
  }

  else if (lineSensorValues[2] >= 900 and lineSensorValues[0] >= 900) // modus sving høyre (men det trengar man ikke til denne løypa)
  {
    on_line = 4; // sving høyre
  }
}

void turningcrossroad()
{ // funksjon når man skal svinge 90 grader til høyre

  resetdistance(); // henter inn resetdistance funksjonen
  while (reset == 0 and encoder.getCountsLeft() <= 400)
  {                                                  // når reset har gjort jobben *****
    motors.setSpeeds(speed * (0.5), speed * (-0.5)); // får bilen til å snu
  }

  if (encoder.getCountsLeft() >= 400 and reset == 0)
  { // når bilen har snudd 90 grader byttes modus, når farten er 400 snus den på 500
    reset = 1;
    previusmilllis_restart = millis();
    speed_1 = speed; // endrer farten tilbake til normal etter endt sving
  }
}

void steeringleft()
{ // sving litt til venstre
  for (int i = 0; i <= 100; i++)
  { // tvinger bilen til å svinge til venstre
    motors.setSpeeds(speed * 0.5, speed * 1);
  }
}
void steeringright()
{ // sving litt til høyre
  for (int i = 0; i <= 100; i++)
  {
    motors.setSpeeds(speed * 1, speed * 0.5);
  }
}

// styring med pid
void steeringPID()
{
  resetdistance();
  left_speed = (map(position, 0, 4000, 0, 100)) / 100.0; // faktor til å gange fart
  right_speed = (map(position, 0, 4000, 100, 0)) / 100.0;

  error = position - 2000;                                  // hvor feil er bilen
  speedDifference = error * 0.25 + (error - lastError) * 3; // deriverte

  lastError = error; // husk hvor bilen var

  left_speed1 = (int16_t)speed_1 * left_speed + speedDifference * left_speed; // legger inn den deriverte
  right_speed1 = (int16_t)speed_1 * right_speed - speedDifference * right_speed;

  left_speed1 = constrain(left_speed1, 0, (int16_t)speed); // hva skal bilen gjøre
  right_speed1 = constrain(right_speed1, 0, (int16_t)speed);

  motors.setSpeeds(left_speed1, right_speed1); // setter fart på motor
}

// styring uten PID
void steeringStandar()
{
  left_speed = (map(position, 0, 4000, 0, 100)) / 100.0;
  right_speed = (map(position, 0, 4000, 100, 0)) / 100.0;

  motors.setSpeeds(left_speed, right_speed);
}

// funksjon for rundetelling (teipet som kryss)
void rounds()
{
  if (lineSensorValues[0] >= 1000 && lineSensorValues[4] >= 1000)
  {
    currentmillis_round = millis();
    if (currentmillis_round - previusmilllis_round >= time_round)
    {
      previusmilllis_round = currentmillis_round;
      rounds_N = rounds_N + 1;
    }
  }
}

//
// hovedmodus:
//
void whatToDoMain()
{                    // hva skal skje i hovedmodus
  readSensorvalue(); // lese sensorverdier
  switch (on_line)
  { // switch funksjon, byttes med hensyn på sensorverdier

  case 0: // modus for å kjøre etter linje
    if (line_value_change == 1)
    { // for å starte wrong way funksjonen i riktig modus.
      line_value_change = 0;
    }
    first = 0;

    steeringPID(); // linjesensor funksjon
    break;

  case 1:
    if (line_value_change == 0)
    {              // første runde går den inn i funkjsonen
      turning = 0; // riktig modus
      reset = 1;
      line_value_change = 1;
    }
    first = 0; // for å starte turning crossroad funksjonen riktig

    wrongway(); // funksjon som gjør at bilen snur når den kjører feil vei

    break;

  case 2:
    if (first == 0)
    { // går inni funksjonen første runde
      reset = 1;
      previusmilllis_restart = millis();
      first = 1;
    }
    turningcrossroad(); // funksjon for 90 graders høyre sving.
    break;

  case 3:
    steeringleft(); // tvinger frem venstresving
    break;

  case 4:
    steeringright(); // tvinger frem høyresving (unødvenig her)

    break;
  }
}

//  bytte hovedmodus
void change_mode()
{

  // hindre debaunce
  buttonState = buttonA.isPressed();
  buttonState_B = buttonB.isPressed();
  if (buttonState != prevButtonState and buttonState_B != prevButtonstate_B)
  {
    prevButtonState = buttonState;
    prevButtonstate_B = buttonState_B;
    if (buttonA.isPressed() and buttonB.isPressed())
    { // hvis A og B trykkes samtidig skifter modus.
      if (mode == 0)
      {
        mode = 1; // hovedmodus
      }
      else if (mode == 1) // lekemodus
      {
        mode = 0;
      }
    }
  }
}

void main_mode()
{ // funksjon som bestemmer modus
  switch (mode)
  {
  case 0: // hovedmodus, der bilen kjører på bane
    // show();
    readValue();
    whatToDoMain();
    rounds();
    change_mode();
    // battery_under5();
    break;

  case 1: // "leke modus" der bilen kjører i firkant runduing og frem og tilbake
    // show();
    button_mode();
    change_mode();
    break;

  default:
    break;
  }
}

//
// bateri:
//

// denne blokken her tar for seg Encoderen. Siden det kan oppstå problemer når encoder når max verdi og går i modus
// er denne funksjonstubben er dedikert til å løse dette problemet.

void encoder1()
{
  batteriet = constrain(batteriet, 0, 100);
  motor_v = encoders.getCountsLeft();  // Henter encoder data
  motor_h = encoders.getCountsRight(); // henter encoder data

  motor_v = motor_v1 + motor_v;
  motor_h = motor_h1 + motor_h;

  // Beregn Counts og Distanse basert på forskjellen mellom nåværende og forrige verdi
  CountsDiffV = motor_v - forrige_motor_v;
  CountsDiffH = motor_h - forrige_motor_h;
  // if blokkene under tar for seg verdiene til encoderen avhengig om den er negativ eller ikke.
  if (CountsDiffV > 30000)
  {
    CountsDiffV -= 65536; // Håndter overgangen til negativ verdi
  }
  else if (CountsDiffV < -30000)
  {
    CountsDiffV += 65536; // Håndter overgangen til positiv verdi
  }
  if (CountsDiffH > 30000)
  {
    CountsDiffH -= 65536; // Håndter overgangen til negativ verdi
  }
  else if (CountsDiffH < -30000)
  {
    CountsDiffH += 65536; // Håndter overgangen til positiv verdi
  }
}

// Funksjon for å få en verdi på knappetrykket A
void knappe_teller_funk()
{
  if (buttonA.getSingleDebouncedPress())
  {
    knappe_teller += 1;
  }
}

// legg opp dette senere

// Denne funksjonstubben handler om å få beregnet  avstand bilen har
void distanse()
{
  encoder1();
  float Counts = ((CountsDiffV + CountsDiffH) / 2);

  float Distanse = Diameter * Pi * Counts / antall_rotasjon; // bergegner lengdekjørt
  charging_constant = Distanse / 0.4;                        // Hvor fort bilen skal lade
  // float lengde_kjort_siden_ladning = Diameter * Pi * Counts / antall_rotasjon; //Beregner lengden kjørt utifra encoderen sine verdier og diameter på beltet

  if (Counts < 0 && batteriet < 100)
  {                                                 // behandler de negative verdiene slik at batteriet ikke går i negative verdi ved negativ counts.
    batteriet = batteriet - charging_constant / 18; // Batteriet går nedover
  }
  else
  {
    batteriet = batteriet - charging_constant / 18; // Batteriet går nedover
  }
  batteriet = constrain(batteriet, 0, 100);

  if (Counts > 0)
  {                                         // Behandler porsitive verdier av Counts
    lengde_kjort = lengde_kjort + Distanse; // Sørger for at distansen
  }
  else
  {
    lengde_kjort = lengde_kjort - Distanse; // Sørger for at distansen ikke blir negativ ved negative counts verdier.
  }
  forrige_motor_h = motor_h;
  forrige_motor_v = motor_v;
}

// Beregner gjennomsnitthastigheten til bilen
void gjennomsnitt_hastighet()
{
  encoder1();
  distanse();
  tid_kjort = millis() / 1000;
  snitt_hastighet = (lengde_kjort) / (tid_kjort);
}

// Tar for seg hva som skjer med batteriet når det går tomt
void batteri_er_tomt()
{
  if (batteriet <= 3)
  { // Hvis batteriet går under 3 prosent så går bilen i lader modus
    previousMillis_bateri = millis();
    modus = 2;
  }
}

void bil_stopper()
{                         // Sender bil til ladning avhengig
  motors.setSpeeds(0, 0); // Setter speeden til 0 slik at den stopper
  // Sjekker om batteriet er under 50. DEtte gjør den slik at den skjønner at den skal lades opp, å ikke gå i kjøre modus.
  // DEn sjekker også om hurtigladning modus er valgt eller ikke.
  if (millis() - previousMillis_bateri >= interval && batteriet <= 50 && knappe_teller % 2 == 1)
  {
    previousMillis_bateri = millis(); // Reseter millis
    modus = 4;                        // Hvis knapp A er trykket blir bilen sendt til hurtigladning
  }

  else if (millis() - previousMillis_bateri >= interval && batteriet <= 50)
  // Hvis knapp ikke er trykket og batteriet er under 50 går den til modus 1 som er lading
  {
    previousMillis_bateri = millis();
    modus = 1; // Hvis knapp A ikke er trykket blir bilen sendt til vanelig ladning
  }
  else if (millis() - previousMillis_bateri >= interval && batteriet >= 90) // Hvis batteriet er over 90 skal den gå til kjøring. For nå vet vi at batteriet har blitt ladet opp
  {
    previousMillis_bateri = millis();
    modus = 3; // Sender bil tilbake til kjøring
  }
}

// Blokken under tar for seg hvordan bilen skal lade ved hurtigladning.

void bil_hurtig_lader()
{
  if (batteriet < 100) // Batteriet er under 100, da skal bilen lade
  {
    motors.setSpeeds(-(speed * 0.5), -(speed * 0.5));  // Setter en ladningshastighet
    knappe_teller = 0;                                 // Reseter knappeteller
    batteriet = batteriet + charging_constant * (-50); // Funksjon for hastigheten til oppladning
    batteriet = constrain(batteriet, 0, 100);
  }
  else if (batteriet >= 100) // når bilen er ladet til 100 prosent skal bilen sendes til modus 2, som gjør den klar for å kjøre igjen
  {
    previousMillis_bateri = millis(); // Reseter previousMillis_bateri
    modus = 2;                        // Sender til stoppefunksjon
  }
}

// Denne funksjonen tar for seg lading av bil i vanlig modus.
void bil_lader()
{
  if (batteriet < 100) // Hvis batteriet er under 100 prosent lades det
  {
    motors.setSpeeds(-(speed * 0.5), -(speed * 0.5)); // Setter en ladehastighet
    batteriet = batteriet + charging_constant * (-6); // Funksjon for hastigheten til oppladning
    batteriet = constrain(batteriet, 0, 100);
  }

  else if (batteriet >= 100) // når batteriet er 100 prosent sendes det til modus 2, som stopper bilen.
  {
    previousMillis_bateri = millis(); // Reseter previousMillis_bateri
    modus = 2;                        // Sender bilen til modus 2 som er stans.
  }
}

void modus_switch()
{
  gjennomsnitt_hastighet(); // Funksjon for gjennomsnittshastighet
  encoder1();               // Funksjonen til encoder
  distanse();               // Funksjonen til distanse måling
  knappe_teller_funk();     // Funksjonen til knappen

  switch (modus)
  {
  case 1: // Dette er modusen som tar for seg oppladning av bil. Oppladningshastigheten er normal.
    bil_lader();
    break;

  case 2: // Modusen som får bilen til å stoppe opp mellom switching av kjøring frem og tilbake, og visa verca.
    bil_stopper();
    break;

  case 3: // Modusen som sørger for at bilen kjører fremover
    main_mode();
    batteri_er_tomt();
    break;

  case 4:
    bil_hurtig_lader(); // Dette er modusen som tar for seg hurtigoppladning av bilen. Oppladningshastigheten er høy.
    break;
  default:
    break;
  }
}

//
// skjerm;
//

// skjermviser
void show()
{
  display.gotoXY(3, 0);
  display.println(batteriet);
  display.gotoXY(5, 1);
  display.println(lengde_kjort);
  display.gotoXY(0, 1);
  display.println(knappe_teller);
  display.gotoXY(5, 3);
  display.println(mode);
 // display.gotoXY(0, 3);
 // display.println(motor_v);
}

void loop()
{
  show(); // skjerm
  // main_mode(); // henter inn modus funksjon, siden vi bare har denne kan vi egt legge den rett inn.
  modus_switch(); // Switchen.
}
