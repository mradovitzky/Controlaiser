//para el motor derecho
int MD2=8;//El pin 8 de arduino se conecta al pin L_EN del puente H derecho
int MD1=9;//El pin 9 de arduino se conecta al pin R_EN del puente H derecho
int PWMD1=10;//El pin 10 de arduino se conecta con el pin RPWM del puente H derecho
int PWMD2=11;//El pin 11 de arduino se conecta con el pin LPWM del puente H derecho
int ED=3;//La señal del encoder derecho ingresa por el pin 3 de arduino (capaz de disparar interrupciones)
//para el motor izquierdo
int MI1=4;//El pin 4 de arduino se conecta al pin L_EN del puente H izquierdo
int MI2=7;//El pin 7 de arduino se conecta al pin R_EN del puente H izquierdo
int PWMI1=5;//El pin 5 de arduino se conecta con el pin RPWM del puente H izquierdo
int PWMI2=6;//El pin 6 de arduino se conecta con el pin LPWM del puente H izquierdo
int EI=2;//La señal del encoder izquierdo ingresa por el pin 2 de arduino (capaz de disparar interrupciones)

//variables de cálculo de velocidad y PWM para el motor derecho
int t1D=0;//t1 y t0 tomarán valores del timer, para poder calcular el delta(t)
int t0D=0;
int dtD=0;//delta(t) entre interrupciones disparadas por el encoder
int nD = 0;//variable auxiliar para calcular delta(t), contando la cantidad de veces que se reinició el timer entre interrupciones disparadas por encoder
volatile unsigned int rpm0D=0;//última velocidad calculada
volatile unsigned int rpm1D=0;//anteúltima velocidad calculada
volatile unsigned int rpm2D=0;//antepenúltima velocidad calculada
int PWM0D=0;//último PWM calculado
int PWM1D=25;//anteúltimo PWM calculado
int PWM2D=0;//antepenúltimo PWM calculado

//variables de cálculo de velocidad y PWM para el motor izquierdo
int t1I=0;//t1 y t0 tomarán valores del timer, para poder calcular el delta(t)
int t0I=0;
int dtI=0;//delta(t) entre interrupciones disparadas por el encoder
int nI = 0;//variable auxiliar para calcular delta(t), contando la cantidad de veces que se reinició el timer entre interrupciones disparadas por encoder
volatile unsigned int rpm0I=0;//última velocidad calculada
volatile unsigned int rpm1I=0;//anteúltima velocidad calculada
volatile unsigned int rpm2I=0;//antepenúltima velocidad calculada
int PWM0I=0;//último PWM calculado
int PWM1I=25;//anteúltimo PWM calculado
int PWM2I=0;//antepenúltimo PWM calculado


//variables de cálculo para el controlador maestro
volatile unsigned int sumaD=0;//cantidad acumulada de pulsos del encoder derecho
volatile unsigned int sumaI=0;//cantidad acumulada de pulsos del encoder iquierdo
volatile unsigned int sumaError=0;//diferencia de pulsos
volatile unsigned int setPointM=40;//set point maestro

int paso = 1;//máximo incremento de PWM posible entre dos instancias de control consecutivas
int setPointD = 40;
int setPointI = 40;





void setup() {

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 1562;
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
  TCCR1B |= (1 << WGM12);   // CTC mode

  pinMode(MI1, OUTPUT);
  pinMode(MI2, OUTPUT);
  pinMode(PWMI1, OUTPUT);
  pinMode(PWMI2, OUTPUT);
  pinMode(MD1, OUTPUT);
  pinMode(MD2, OUTPUT);
  pinMode(PWMD1, OUTPUT);
  pinMode(PWMD2, OUTPUT);

    noInterrupts();           // disable all interrupts
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(EI), readEncoderI, RISING);
    attachInterrupt(digitalPinToInterrupt(ED), readEncoderD, RISING);
    interrupts();             // enable all interrupts

  digitalWrite(6,LOW);
}


void loop() {
 
  digitalWrite(8,HIGH);
  digitalWrite(9,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(7,HIGH);
 
}


void readEncoderI(){

  if (sumaI = 32760) {
      sumaD = 0;
      sumaI = sumaError;
  }//reinicia los contadores antes de que las variables lleguen a su máxima capacidad

  sumaI=sumaI+1;
  
  t1I=TCNT1;

  dtI=(nI*1562)+t1I-t0I;

 
  t0I=t1I;
  rpm0I=((-1)*(60/20)*15625/dtI)/7.3; //60 segundos, 20 ranuras del enconder, 15625 pulsos por segundo, 7,3 relación de reducción piñón-encoder
  nI=0;

  rpm2I=rpm1I;
  rpm1I=rpm0I;

}
         

void readEncoderD(){

  if (sumaD = 32760) {
      sumaD = 0;
      sumaI = sumaError;
  }//reinicia los contadores antes de que las variables lleguen a su máxima capacidad
  
  sumaD=sumaD+1;
  
  t1D=TCNT1;

  dtD=(nD*1562)+t1D-t0D;

 
  t0D=t1D;
  rpm0D=((-1)*(60/20)*15625/dtD)/7.3; //60 segundos, 20 ranuras del enconder, 15625 pulsos por segundo, 7,3 relación de reducción piñón-encoder
  nD=0;

  rpm2D=rpm1D;
  rpm1D=rpm0D;

}

ISR( TIMER1_COMPA_vect )          // timer compare interrupt service routine
{

//Controlador maestro///////////////////////////////////////////////////////////////////////////

sumaError = sumaI - sumaD;

setPointI = setPointM - (sumaError * 10);
setPointD = setPointM + (sumaError * 10);

///////////////////////////////////////////////////////////////////////////////////////////////

nI=nI+1;
//tI=tI+1;
nD=nD+1;
//tD=tD+1;

//Cálculo del PWM para el motor izquierdo

PWM0I = (setPointI-rpm0I)*5;

if (PWM0I > 120) {
  PWM0I=120;
}

if (PWM0I < 25) {
  PWM0I=25;
}

if (PWM0I > PWM1I + paso){//el incremento de PWM debe ser controlado
  PWM0I=PWM1I + paso;
}

if (PWM0I < PWM1I + paso){//el incremento de PWM debe ser controlado
  PWM0I=PWM1I - paso;
}


analogWrite(6,PWM0I);

PWM2I=PWM1I;
PWM1I=PWM0I;


//Cálculo del PWM para el motor derecho

//PWM0D = 1.997*PWM1D - 0.997*PWM2D + 0.2714*(setPoint-rpm0D) - 0.5391*(setPoint-rpm1D) + 0.2677*(setPoint-rpm2D);
//PWM0D = PWM1D + 12*(setPoint-rpm0D) - 12*(setPoint-rpm1D);
PWM0D = (setPointD-rpm0D)*5;

if (PWM0D > 120) {
  PWM0D=120;
}

if (PWM0D < 25) {
  PWM0D=25;
}

if (PWM0D > PWM1D + paso){//el incremento de PWM debe ser controlado
  PWM0D=PWM1D + paso;
}

if (PWM0D < PWM1D + paso){//el incremento de PWM debe ser controlado
  PWM0D=PWM1D - paso;
}


analogWrite(11,PWM0D);

PWM2D=PWM1D;
PWM1D=PWM0D;

}
