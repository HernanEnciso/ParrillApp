#include <OneWire.h>                        //LIBRERIA DE SENSOR TEMPERATURA
#include <DallasTemperature.h>              //LIBRERIA DE SENSOR TEMPERATURA
#include <SoftwareSerial.h>                 //PARA EL MAPEO DE PINES Tx Y Rx DEL BLUETOOTH
#include <String.h>
#define pinTemp 2                           //PIN SENSOR TEMPERATURA
#define entradaINFRA A3                     //SENSOR INFRARROJO
OneWire ourWire(pinTemp);                   //Se establece el pin declarado como bus para la comunicación OneWire
DallasTemperature sensors(&ourWire);        //Se llama a la librería DallasTemperature
SoftwareSerial BTserial(8,12); // RX | TX     MAPEO DE PINES Tx Y Rx DE BLUETOOTH


const int pinLluvia = 9;                    //PIN SENSOR LLUVIA
//------------------VARIABLES MILLIS --------------------------------------------------------
unsigned long previousMillisLLUVIA = 0;
unsigned long previousMillisINFRA = 0;
unsigned long previousMillisTEMP = 0;
unsigned long previousMillisULTRASONIDO = 0;
unsigned long previousMillisBUZZ = 0;
unsigned long previousMillisRGB = 0;
//------------------INTERVALOS---------------------------------------------------------------
const long intervaloLluvia = 1000;
const long intervaloINFRA = 500;
const long intervaloTemperatura = 1000;
const long intervaloULTRASONIDO = 1000;
const long intervaloBuzzer = 200; 
const long intervaloRGB = 200;
//------------------------VARIABLES PARA LOS SENSORES----------------------------------------
//ULTRASONIDO
const int trigger_us100 = 4;             
const int echo_us100 = 5;                 
//TEMPERATURA
float temperatura_maxima = 35;            //TEMPERATURA MAXIMA
float temperatura_minima = 20;            //TEMPERATURA MINIMA
float temperatura_actual = 0;             //TEMPERATURA ACTUAL MEDIDA POR EL SENSOR
//VARIABLES GLOBALES PARA LA PARRILLA
float alturaActualParrilla = 10;          //ALTURA ACTUAL DE LA PARRILLA SEGÚN SENSOR ULTRASONIDO
const float topeAlturaParrilla = 20;      //ALTURA MÁXIMA DE LA PARRILLA
const float pisoParrilla = 6;             //ALTURA BASE DE LA PARRILLA (EL 0)
const int error_altura = 1;               //UMBRAL DE ACEPTACION DE LA ALTURA DE LA PARRILLA (+- cm)
float alturaDeseada ;                     //ALTURA A DONDE SE DESEA MOVER LA PARRILLA  
float desplazamiento = 0;                 //ALTURA QUE SE TIENE QUE SUBIR O BAJAR, ENVIADA POR LA APLICACION
//------------------------VARIABLES PARA LOS ACTUADORES----------------------------------------
//BUZZER
unsigned int encendido=1;                 
//LED RGB
unsigned int flag_ON_OFF = 0;             
//MOTOR
const int arriba = 7;                     
const int abajo = 6;                      
//------------------FLAGS------------------------------------------------------------------
char FLAG_LLUVIA = 0;                     //SI ESTA ACTIVADO EL SENSOR DE LLUVIA
char FLAG_TEMPERATURA = 0;                //SI ESTA ACTIVADO EL SENSOR DE TEMPERATURA
char FLAG_INFRARROJO = 0;                 //SI ESTA ACTIVADO EL SENSOR INFRARROJO
char MODO_INFRARROJO = 0;                 //SI ESTA ACTIVADO ESTE MODO, EL SENSOR DE TEMPERATURA NO ESTÁ ACTIVADO
char FLAG_MODO_BT = 0;                    //MODO_AUTOMATICO=0 ; MODO_MANUAL=1
char FLAG_INICIAL = 0;                    //PARA QUE ENTRE EL MODO AUTOMATICO LA PARRILLA AL INICIAR EL SKETCH
char FLAG_SHAKE = 0;                      //PARA QUE SI SE UTILIZO EL SHAKE, EL MOTOR SE MUEVA A LA ALTURA DESEADA
char comprobacion = 0;                    //SI HAY QUE LEER POR EL BLUETOOTH
char RECEPCION_ACCION_LLUVIA_TEMP = 0;    //PARA PODER APAGAR EL LED SI SE APAGAR POR MODO MANUAL
char RECEPCION_ACCION_INFRARROJO = 0;     //PARA PODER APAGAR EL BUZZER SI SE APAGAR POR MODO MANUAL
//------------------------PROTOCOLO DE TRANSMISION-----------------------------------------
//------------------------MENSAJES QUE RECIBE ARDUINO--------------------------------------
char MODO_MANUAL[2]        = "m";
char MODO_AUTOMATICO[2]    = "a";
char APAGAR_LED[2]         = "L";
char APAGAR_BUZZER[2]      = "B";
char BAJAR_MOTOR[6];       
char SUBIR_MOTOR[6];
char SETEAR_VALORES_TEMPERATURA[11];
//------------------------MENSAJES QUE ENVIA ARDUINO---------------------------------------
char PROXIMIDAD_NO_DETECTADA[3]           = "P0";
char PROXIMIDAD_DETECTADA[3]              = "P1";
char LLUVIA_NO_DETECTADA[3]               = "R0";
char LLUVIA_DETECTADA[3]                  = "R1";
char BUZZER_APAGADO[3]                    = "B0";
char BUZZER_PRENDIDO[3]                   = "B1";
char LED_APAGADO[3]                       = "L0";
char LED_PRENDIDO_ROJO[3]                 = "L1";
char LED_PRENDIDO_AZUL[3]                 = "L2";
char LED_PRENDIDO_MORADO[3]               = "L3";
char TEMPERATURA_MAXIMA_SUPERADA[3]       = "W1";
char TEMPERATURA_MINIMA_INSUFICIENTE[3]   = "W2";
char TEMPERATURA_OK[3]                    = "W0";
char TEMPERATURA_ACTUAL_ES[6];                        //CONSIDERAR QUE ESTAMOS MANDANDO NUMEROS ENTEROS, NO float's...
char ALTURA_ACTUAL_ES[6];                             //IDEM "TEMPERATURA_ACTUAL_ES"
//------------------------VARIABLES PARA EL BLUETOOTH--------------------------------------
char c = ' ';
char CADENA_ANALIZABLE[30] = "";
int i = 0;
int CONFIGURACION_BLUETOOTH = 20;     //20=MODO_AUTOMATICO ; 10=MODO_MANUAL ; 30=APAGAR_LED ; 40=APAGAR_BUZZER ; 50=SHAKE_MOTOR(SUBIR O BAJAR) ; 60=SETEAR_RANGOS_TEMPERATURA
//------------------FIN DE VARIABLES GLOBALES----------------------------------------------
//------------------SETUP------------------------------------------------------------------
void setup() {
  Serial.begin(9600);                   //iniciar puerto serie
  BTserial.begin(9600);                 //SETUP DE VELOC. DE TRANSMISION DEL MOD. BLUETOOTH

  //SETUP SENSOR LLUVIA
  pinMode(A0, INPUT);                   //definir pin como entrada

  //SETUP SENSOR INFRARROJO
  pinMode(entradaINFRA,INPUT);
  pinMode(11,OUTPUT);
  digitalWrite(A2,HIGH);
  digitalWrite(A1,LOW);

  //SETUP SENSOR ULTRASONIDO
  pinMode(trigger_us100, OUTPUT);
  pinMode(echo_us100, INPUT);

  //SETUP LED RGB
  pinMode(A4,OUTPUT);
  pinMode(A5,OUTPUT);
  pinMode(10,OUTPUT);  
}
 //------------------LOOP------------------------------------------------------------------
void loop(){
  leerBT();
  if (comprobacion == 1){                         //SI LLEGÓ UN '\n'...
    CONFIGURACION_BLUETOOTH = analizarCadena();
    comprobacion = 0;
  }
  if(FLAG_INICIAL == 0){
    modoAutomatico();
  }
  if(CONFIGURACION_BLUETOOTH == 10){
    FLAG_MODO_BT = 1;
    FLAG_INICIAL=1;
    //modoManual();
  }
  
  if (CONFIGURACION_BLUETOOTH == 20){
    FLAG_MODO_BT = 0;
    RECEPCION_ACCION_LLUVIA_TEMP =0;
    RECEPCION_ACCION_INFRARROJO = 0;
    modoAutomatico();
  }
  
  //MODO MANUAL
  if(FLAG_MODO_BT == 1){
    sensar();
    if (FLAG_SHAKE == 1 ){
      moverMotor(alturaDeseada);
    }
  }
  if (FLAG_MODO_BT == 1 && CONFIGURACION_BLUETOOTH == 30){
    apagarRGB();
    enviarEstadoActualAANDROID(LED_APAGADO);
    RECEPCION_ACCION_LLUVIA_TEMP = 1;
    CONFIGURACION_BLUETOOTH = 10;                           //DEBE SEGUIR EN EL MODO MANUAL AUNQUE NO RECIBA NINGUN DATO.. PORQUE ESTOY DENTRO DE LA APLICACION
  }
  if (FLAG_MODO_BT == 1 && CONFIGURACION_BLUETOOTH == 40){
    apagarBuzzer();
    enviarEstadoActualAANDROID(BUZZER_APAGADO);
    RECEPCION_ACCION_INFRARROJO = 1;
    CONFIGURACION_BLUETOOTH = 10;
  }
  if (FLAG_MODO_BT == 1 && CONFIGURACION_BLUETOOTH == 50){
    enviarAlturaActualAANDROID();
    CONFIGURACION_BLUETOOTH = 10;
  }
  if (FLAG_MODO_BT == 1 && CONFIGURACION_BLUETOOTH == 60){
    //LOS VALORES DE TEMPERATURAS MINIMA Y MAXIMA YA SE SETEARON ACÁ. DEBE SEGUIR EN MODO MANUAL
    enviarTemperaturaActualAANDROID();
    CONFIGURACION_BLUETOOTH = 10;
  }
}

void modoAutomatico(){
  sensar();
}

void sensar(){
  unsigned long currentMillis = millis();
  
  sensarLluvia(currentMillis);          //SENSADO DE LA LLUVIA
  
  sensarINFRARROJO(currentMillis);      //SENSADO DE PROXIMIDAD

  if(MODO_INFRARROJO == 0){
    sensarTemperatura(currentMillis);   //SENSADO DE TEMPERATURA
  }
  
  sensarUltrasonido(currentMillis);     //SENSADO DE DISTANCIA (SENSOR DE ULTRASONIDO)
}
 //------------------FUNCIONES DE LOS SENSORES------------------------------------------------------------------
void sensarLluvia(unsigned long currentMillis){
  int value = 0;
  if (currentMillis - previousMillisLLUVIA >= intervaloLluvia) {
    previousMillisLLUVIA = currentMillis;
    value = digitalRead(pinLluvia);     //lectura del sensor de lluvia 
    if (value == LOW){
      FLAG_LLUVIA = 1;
      if (RECEPCION_ACCION_LLUVIA_TEMP == 0){
        Serial.println("LLUVIA DETECTADA");
        prenderAZUL(currentMillis);
      }
      if (FLAG_MODO_BT == 0 && FLAG_TEMPERATURA == 0){
         RECEPCION_ACCION_LLUVIA_TEMP = 0;  
      }
      if (FLAG_MODO_BT == 1){             //SI ESTÁ EN EL MODO MANUAL..
        enviarEstadoActualAANDROID(LLUVIA_DETECTADA);
      }
    }
    else{
      FLAG_LLUVIA = 0;
      //RECEPCION_ACCION_LLUVIA_TEMP = 0;  
      if (FLAG_MODO_BT == 0){           //SI ESTA EN EL MODO AUTOMATICO..
        apagarRGB();
      }
      if (FLAG_MODO_BT == 1){
        enviarEstadoActualAANDROID(LLUVIA_NO_DETECTADA);
      }
      
      if(FLAG_TEMPERATURA == 0){
          RECEPCION_ACCION_LLUVIA_TEMP = 0;  
      }
    }
  }
}

void sensarTemperatura(unsigned long currentMillis) {
  if (currentMillis - previousMillisTEMP >= intervaloTemperatura) {  
    previousMillisTEMP = currentMillis;
    sensors.requestTemperatures();       //PREPARA EL SENSOR PARA LA LECTURA
    temperatura_actual = sensors.getTempCByIndex(0);

    if ( temperatura_actual > temperatura_maxima){     
      FLAG_TEMPERATURA = 1;
      if(!llegoATopeMAXIMO() ){           //SI LA TEMPERATURA ES SUPERIOR A LA MAXIMA Y LA PARRILLA NO ESTÁ EN EL TOPE MAXIMO...
        if (FLAG_MODO_BT == 0){
          Serial.print("TEMPERATURA MAXIMA SUPERADA: ");
          subirMotor();
        }
      } else {                            //SI LA TEMPERATURA ES SUPERIOR A LA MAXIMA Y LA PARRILLA ESTÁ EN EL TOPE MAXIMO...
         if (FLAG_MODO_BT == 0){
          Serial.println("PARRILLA LLEGO AL TOPE MAXIMO");
          frenarMotor();
        }
      }
      if (RECEPCION_ACCION_LLUVIA_TEMP == 0){
        prenderROJO(currentMillis);
      }
      if (FLAG_MODO_BT == 0 && FLAG_LLUVIA == 0){
         RECEPCION_ACCION_LLUVIA_TEMP = 0;  
      }
      if (FLAG_MODO_BT == 1){             //SI ESTÁ EN EL MODO MANUAL..
        enviarEstadoActualAANDROID(TEMPERATURA_MAXIMA_SUPERADA);
      }
    }

    if( temperatura_minima > temperatura_actual){
      FLAG_TEMPERATURA = 1;
      if(!llegoATopeMINIMO()){            //SI LA TEMPERATURA ES INFERIOR A LA MINIMA Y LA PARRILLA NO ESTÁ EN EL TOPE MINIMO (O "PISO")...
        if (FLAG_MODO_BT == 0){
          Serial.print("TEMPERATURA MINIMA INSUFICIENTE: ");
          bajarMotor();
        }
      } else {                        //SI LA TEMPERATURA ES INFERIOR A LA MINIMA Y LA PARRILLA ESTÁ EN EL TOPE MINIMO (O PISO)...
          if (FLAG_MODO_BT == 0){
            Serial.println("PARRILLA LLEGO AL TOPE MINIMO");
            frenarMotor();
          }
      }
      
      if (RECEPCION_ACCION_LLUVIA_TEMP == 0){
        prenderROJO(currentMillis);
      }
      
      if (FLAG_MODO_BT == 0 && FLAG_LLUVIA == 0){
         RECEPCION_ACCION_LLUVIA_TEMP = 0;  
      }
      
      if (FLAG_MODO_BT == 1){             //SI ESTÁ EN EL MODO MANUAL..
        enviarEstadoActualAANDROID(TEMPERATURA_MINIMA_INSUFICIENTE);
      }
    }

    //TEMPERATURA DENTRO DE LOS RANGOS MINIMO Y MAXIMO...
    if (temperatura_actual >= temperatura_minima && temperatura_actual <= temperatura_maxima){
      FLAG_TEMPERATURA = 0;
      //RECEPCION_ACCION_LLUVIA_TEMP = 0;  
      if (FLAG_MODO_BT == 0){
        apagarRGB();
        if (FLAG_INFRARROJO == 0){
          frenarMotor();
        }
      }
      if (FLAG_MODO_BT == 1){
        enviarEstadoActualAANDROID(TEMPERATURA_OK);
      }
      if(FLAG_LLUVIA == 0){
        RECEPCION_ACCION_LLUVIA_TEMP = 0;  
      }
    }
    Serial.print(temperatura_actual);
    Serial.println(" ºC");
 }
}

void sensarINFRARROJO(unsigned long currentMillis) {
  if (currentMillis - previousMillisINFRA >= intervaloINFRA) {
    previousMillisINFRA = currentMillis;
    if(analogRead(entradaINFRA)<500){
      FLAG_INFRARROJO = 1;
      MODO_INFRARROJO = 1;
      digitalWrite(11,HIGH);
      if (FLAG_MODO_BT == 0){                               //  EL MOTOR SE VA A MOVER HASTA EL TOPE SI ES QUE ESTÁ EN MODO AUTOM. SINO SE QUEDA DONDE ESTÁ PERO SUENA EL BUZZER
        if(topeAlturaParrilla < alturaActualParrilla){
          frenarMotor();
        }
        else{
          subirMotor();
        }
      }
      if (FLAG_MODO_BT == 1){             //SI ESTÁ EN EL MODO MANUAL..
        enviarEstadoActualAANDROID(PROXIMIDAD_DETECTADA);
        frenarMotor();
      }

      if (RECEPCION_ACCION_INFRARROJO == 0){
        sonarBuzzer(currentMillis);
      }
    }
    else{                                       // SI YA NO HAY OBSTÁCULO....
      FLAG_INFRARROJO = 0;
      MODO_INFRARROJO = 0;
      RECEPCION_ACCION_INFRARROJO = 0;
      if (FLAG_MODO_BT == 0){
        apagarBuzzer();
      }
      if (FLAG_MODO_BT == 1){
        enviarEstadoActualAANDROID(PROXIMIDAD_NO_DETECTADA);
      }
      digitalWrite(11,LOW);
    }
  }
}

/*void sensarINFRARROJO(unsigned long currentMillis) {
  if (currentMillis - previousMillisINFRA >= intervaloINFRA) {
    previousMillisINFRA = currentMillis;
    if(analogRead(entradaINFRA)<500){
      FLAG_INFRARROJO = 1;
      MODO_INFRARROJO = 1;
      digitalWrite(11,HIGH);
      //  Serial.println("OBSTACULO");
      if (FLAG_MODO_BT == 0){                               //  EL MOTOR SE VA A MOVER HASTA EL TOPE SI ES QUE ESTÁ EN MODO AUTOM. SINO SE QUEDA DONDE ESTÁ PERO SUENA EL BUZZER
        if(topeAlturaParrilla < alturaActualParrilla){
          frenarMotor();
        }
        else{
          subirMotor();
        }
      }
      if (RECEPCION_ACCION_INFRARROJO == 0){
        sonarBuzzer(currentMillis);
      }
      if (FLAG_MODO_BT == 1){             //SI ESTÁ EN EL MODO MANUAL..
        enviarEstadoActualAANDROID(PROXIMIDAD_DETECTADA);
      }
    }
    else{                                       // SI YA NO HAY OBSTÁCULO....
      FLAG_INFRARROJO = 0;
      MODO_INFRARROJO = 0;
      RECEPCION_ACCION_INFRARROJO = 0;
      if (FLAG_MODO_BT == 1){
        enviarEstadoActualAANDROID(PROXIMIDAD_NO_DETECTADA);
      }
      digitalWrite(11,LOW);
      if (FLAG_MODO_BT == 0){
        apagarBuzzer();
      }
    }
  }
}*/

void sensarUltrasonido(unsigned long currentMillis)
{
  if (currentMillis - previousMillisULTRASONIDO >= intervaloULTRASONIDO) {
    previousMillisULTRASONIDO = currentMillis;
  
    alturaActualParrilla = distanciaDeLaParrilla(trigger_us100, echo_us100);
    // Devuelve el resultado vía UART
    Serial.print("Distancia: ");
    Serial.print(alturaActualParrilla);
    Serial.println("cm");
  }
}
//------------------FUNCIONES DE LOS ACTUADORES------------------------------------------------------------------
//BUZZER
void sonarBuzzer(unsigned long currentMillis){
  int intensidad;
  if (currentMillis - previousMillisBUZZ >= intervaloBuzzer) {
    previousMillisBUZZ = currentMillis;
    
    if (encendido==0){ //si está apagado el buzzer...  
      encendido=1;
      intensidad=20;
    }
    else{
      encendido=0;
      intensidad=0;
    }
      analogWrite(3, intensidad);
  }
}

void apagarBuzzer(){
   analogWrite(3,0);
}
//LED
void prenderAZUL(unsigned long currentMillis){

  int intensidadRGB;
  if (currentMillis - previousMillisRGB >= intervaloRGB) {
    previousMillisRGB = currentMillis;
    if (flag_ON_OFF == 0){
      flag_ON_OFF = 1;
      intensidadRGB = 255;
    }
    else{
      flag_ON_OFF = 0;
      intensidadRGB = 0;
    }
    //Color(0,intensidadRGB,0);
    analogWrite(A4,intensidadRGB);
  }
}

void prenderROJO(unsigned long currentMillis){

  int intensidadRGB;
  if (currentMillis - previousMillisRGB >= intervaloRGB) {
    previousMillisRGB = currentMillis;
    if (flag_ON_OFF == 0){
      flag_ON_OFF = 1;
      intensidadRGB = 255;
    }
    else{
      flag_ON_OFF = 0;
      intensidadRGB = 0;
    }
    //Color(0,intensidadRGB,0);
    analogWrite(A5,intensidadRGB);
  }
}

void apagarRGB(){
  Color(0,0,0);
}

void Color(int R, int G, int B){
  analogWrite(A4 , B) ;
  analogWrite(10, G) ;
  analogWrite(A5, R) ;
}  
//MOTOR
void subirMotor(){
  digitalWrite(arriba,HIGH);
  digitalWrite(abajo,LOW);
}

void bajarMotor(){
  digitalWrite(arriba,LOW);
  digitalWrite(abajo,HIGH);
}

void frenarMotor(){
  digitalWrite(arriba,LOW);
  digitalWrite(abajo,LOW);
}

void moverMotor(float altura) {
  float cota_sup = altura + error_altura;
  float cota_inf = altura - error_altura;
  if ((cota_sup >= alturaActualParrilla) && (alturaActualParrilla >= cota_inf)){
    FLAG_SHAKE = 0;
    frenarMotor();
  }
  if (alturaActualParrilla > cota_sup && alturaActualParrilla > pisoParrilla){
    bajarMotor();
  }
  if(cota_inf > alturaActualParrilla && topeAlturaParrilla > alturaActualParrilla){
    subirMotor();
  }
}

float distanciaDeLaParrilla(int trigger, int echo)     //Devuelve la distancia de la parrila con el piso en cm (centimetros)
{
  // Trigger inicial
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
 
  // Comienzo de la medida
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
 
  // Adquisición y conversión a metros
  float distancia = pulseIn(echo, HIGH);
  distancia = distancia * 0.01657;
  return distancia;
}

int llegoATopeMAXIMO(){
  if ((topeAlturaParrilla + error_altura) >= alturaActualParrilla && alturaActualParrilla >= (topeAlturaParrilla - error_altura)){
    return 1;
  }
  else return 0;
}

int llegoATopeMINIMO(){
  if ((pisoParrilla + error_altura) >= alturaActualParrilla && alturaActualParrilla >= (pisoParrilla - error_altura)){
    return 1;
  }
  else return 0;
}
//--------------------------------FUNCIONES DE BLUETOOTH---------------------------------------------------------
//ENVIAR ESTADOS A LA APLICACION
void enviarEstadoActualAANDROID(char* cadAux){
    while (*cadAux != '\0'){
        BTserial.write(*cadAux);
        cadAux++;
    }
    BTserial.write('\n');
}

void enviarTemperaturaActualAANDROID(){
  //float valor = 13.5;
  TEMPERATURA_ACTUAL_ES[0] = 'T';
  itoa((int)temperatura_actual, TEMPERATURA_ACTUAL_ES + 1,10);
  enviarEstadoActualAANDROID(TEMPERATURA_ACTUAL_ES);
}

void enviarAlturaActualAANDROID(){
  ALTURA_ACTUAL_ES[0] = 'A';
  itoa((int)alturaActualParrilla, ALTURA_ACTUAL_ES + 1,10);
  enviarEstadoActualAANDROID(ALTURA_ACTUAL_ES);
}

//LEER DEL BT
void leerBT(){
  if (BTserial.available()){
    c = BTserial.read();
    CADENA_ANALIZABLE[i] = c;
    i++;
    if (c == '\n'){
      //Serial.print(CADENA_ANALIZABLE);
      comprobacion = 1;
    }
  }
}

int analizarCadena(){
  
  if ( strstr(CADENA_ANALIZABLE, MODO_MANUAL) != 0 ){
     Serial.println("ENTRANDO A MODO MANUAL.");
     //ENTRAR AL MODO MANUAL...
     BTserial.write('\r');                             //'\r' ES EL RETORNO DE CARRO
     limpiarCadena();
     return 10;                                              //ESTÁ HECHO PARA QUE SI CUMPLE ÉSTA CONDICIÓN, NO PREGUNTE POR LOS DEMAS "if".
  }

  if ( strstr(CADENA_ANALIZABLE, MODO_AUTOMATICO) != 0 ){
    Serial.println("ENTRANDO A MODO AUTOMATICO.");
    //ENTRAR AL MODO AUTOMATICO (LO QUE PROGRAMAMOS EN EL OTRO SKETCH)...
    BTserial.write('\r');                             //'\r' ES EL RETORNO DE CARRO
    limpiarCadena();
    return 20;
  }
  
  if ( strstr(CADENA_ANALIZABLE, APAGAR_LED) != 0 ){      //"L" = Apagar LED
    //apagarRGB();
    Serial.println("APAGANDO LED.");
    BTserial.write('\r');                             //'\r' ES EL RETORNO DE CARRO
    limpiarCadena();
    return 30;
  }                 
  
  if ( strstr(CADENA_ANALIZABLE, APAGAR_BUZZER) != 0 ){   //"B" = Apagar Buzzer
   //apagarBuzzer();
     Serial.println("APAGANDO BUZZER.");
     BTserial.write('\r');                             //'\r' ES EL RETORNO DE CARRO
     limpiarCadena();
     return 40;
  }

  if(shakeMotor() != 0){
      moverMotorPorShake();    
      BTserial.write('\r');                             //'\r' ES EL RETORNO DE CARRO
      FLAG_SHAKE = 1;
      limpiarCadena();
      return 50;  
  }

  //EJEMPLO de SETEAR_TEMP: T20.5_30.4;
  if (leerRangosTemperatura() != 0){
    Serial.println("RANGOS DE TEMPERATURA SETEADOS.");
    BTserial.write('\r');                             //'\r' ES EL RETORNO DE CARRO
    limpiarCadena();
    return 60;
  }
       
  BTserial.write('\r');                             //'\r' ES EL RETORNO DE CARRO
  limpiarCadena();
}
//MOVER EL MOTOR POR EL SHAKE
int shakeMotor(){                                 //SI SE RECIBIO UN SUBIR O BAJAR POR EL SHAKE

  if (CADENA_ANALIZABLE[0] == 'b' || CADENA_ANALIZABLE[0] == 's'){
      return 1;
  }
  return 0;
}

void moverMotorPorShake(){                       //b5.0 bajar 5cm || s7.5 subir 7.5 cm

    desplazamiento = getDesplazamiento();

    if(CADENA_ANALIZABLE[0] =='b' ){                      //"b" INDICA QUE DEBE BAJAR EL MOTOR
      Serial.print("BAJANDO MOTOR: ");
      alturaDeseada = alturaActualParrilla-desplazamiento;
      moverMotor(alturaDeseada);

    } else if(CADENA_ANALIZABLE[0] =='s' ){               //"s" INDICA QUE DEBE SUBIR EL MOTOR
      Serial.print("SUBIENDO MOTOR: ");
      alturaDeseada = alturaActualParrilla+desplazamiento;
      moverMotor(alturaDeseada);
    }

    Serial.print(desplazamiento);
    Serial.println(" cms.");
}

float getDesplazamiento(){                        //OBTENER EL DESPLAZAMIENTO DE SUBIR O BAJAR EL MOTOR

    char aux[5] = "";
    for (int a = 1 ; a < 6; a++){                 //VER (CAPAZ ES a < 5)
      aux[a-1] = CADENA_ANALIZABLE[a];
    }
    return (float)atof(aux);
}

int leerRangosTemperatura(){
    i = 0;
    if(CADENA_ANALIZABLE[i] == 'T'){ 
      char aux[5] = "";  
      i++;
          
      for(int a=1 ; a<5; a++){        //T20.5_30.0 || T1_9 || T1.0_10
        aux[a-1] = CADENA_ANALIZABLE[a];
        i++;
      }
      temperatura_minima = (float)atof(aux);
      if (CADENA_ANALIZABLE[i] == '_'){
        i++;
        for(int a=6 ; a<10; a++){
          aux[a-6] = CADENA_ANALIZABLE[a];
          i++;
        }
        temperatura_maxima = (float)atof(aux);
      }
      Serial.print("TEMP MINIMA: ");
      Serial.println(temperatura_minima);
      Serial.print("TEMP MAXIMA: ");
      Serial.println(temperatura_maxima);
      return 1;
    }
    else return 0;
}

//Limpia la cadena analizable
void limpiarCadena(){
  for (int cl=0; cl<=i; cl++){
    CADENA_ANALIZABLE[cl]=0;
  }
  i=0;
}