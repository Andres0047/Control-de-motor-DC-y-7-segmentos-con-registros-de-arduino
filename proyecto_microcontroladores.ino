#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// Contraseñas
const int shortPass[2] = {1,9};
const int longPass[4] = {4,3,2,1};

// Listas para comparar con las contraseñas
int valShortPass[2] = {};
int valLongPass[4] = {};

uint8_t count1 = 0;  // Para asegurar el límite de caractéres a introducir (contraseña corta)
uint8_t count2 = 0;  // Para asegurar el límite de caractéres a introducir (contraseña larga)

#define ROWS 4
#define COLS 4

// Definir los pines del teclado
char keys[ROWS][COLS] = {
	{'1','2','3','A'},
	{'4','5','6','B'},
	{'7','8','9','C'},
	{'*','0','#','D'}
};

uint8_t rowPins[ROWS] = {7,6,5,4}; // Filas conectadas a estos pines
uint8_t colPins[COLS] = {3,2,0,1}; // Columnas conectadas a estos pines

//                                         gfedcba
const uint64_t numbersDisplayAnode[10] = {0b1000000,     //0
                                          0b1111001,     //1
                                          0b0100100,     //2
                                          0b0110000,     //3
                                          0b0011001,     //4
                                          0b0010010,     //5
                                          0b0000010,     //6
                                          0b1111000,     //7
                                          0b0000000,     //8
                                          0b0010000};    //9

//                                           dg
const uint64_t numbersDisplayAnode1[10] = {0b10,     //0
                                           0b00,     //1
                                           0b11,     //2
                                           0b11,     //3
                                           0b01,     //4
                                           0b11,     //5
                                           0b11,     //6
                                           0b00,     //7
                                           0b11,     //8
                                           0b11};    //9


// Función para inicializar el teclado
void keypad_init() {
	// Configurar las filas como salidas y las columnas como entradas con pull-up
	for (int i = 0; i < ROWS; i++) {
		DDRD |= (1 << rowPins[i]); // Configurar fila como salida
		PORTD |= (1 << rowPins[i]); // Configurar fila en alto
	}
	for (int i = 0; i < COLS; i++) {
		DDRD &= ~(1 << colPins[i]); // Configurar columna como entrada
		PORTD |= (1 << colPins[i]); // Activar pull-up interno en columna
	}
	//7 segmentos
	DDRB |= 0xff;
	// DDRC |= 0xff;
  DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
}

// Función para escanear el teclado
char keypad_scan() {
	char key_pressed = 0;

	// Escaneo de filas
	for (int i = 0; i < ROWS; i++) {
		// Configurar la fila actual en bajo
		PORTD &= ~(1 << rowPins[i]);

		// Leer las columnas
		for (int j = 0; j < COLS; j++) {
			// Si se detecta un botón presionado
			if (!(PIND & (1 << colPins[j]))) {
				// Esperar un momento para evitar el rebote
				_delay_ms(50);
				// Verificar nuevamente si el botón sigue presionado
				if (!(PIND & (1 << colPins[j]))) {
					key_pressed = keys[i][j]; // Guardar la tecla presionada
					// Esperar hasta que se libere el botón
					while (!(PIND & (1 << colPins[j])));
				}
			}
		}
		// Restaurar la fila a alto
		PORTD |= (1 << rowPins[i]);
	}

	return key_pressed;
}

int main(void) {

  /* ------------>       CONFIGURACIÓN DEL MOTOR  <------------        */
  // Configuración del motor pin como salida
  DDRB |= (1 << DDB3); // Configurar pin ~11 (PB3) como salida (OC2A)

  // Configurar Timer 2 para PWM
  TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Configurar PWM no invertido en OC2A (pin 11), modo de operación PWM rápido (WGM20 y WGM21)
  TCCR2B = (1 << CS20); // Preescalador de 1 (sin prescaler)
  // Serial.begin(9600);

  // Iniciar el motor con velocidad mínima (apagado)
  OCR2A = 0;

  /* ************************************************************** */

	// Inicializar el teclado
	keypad_init();

	// Inicializar puerto serie para comunicación
	UBRR0 = 103; // Configurar velocidad de transmisión a 9600 bps
	UCSR0B |= (1 << TXEN0); // Habilitar transmisor
	uint8_t number = 0;

	while (1) {
		// Escanear el teclado
		char key = keypad_scan();

		// Si se presiona una tecla, enviarla por el puerto serie
		if (key != 0) {
			number = atoi(&key);
			while (!(UCSR0A & (1 << UDRE0))); // Esperar a que el registro de transmisión esté vacío
			PORTB &= 0x00;
			// PORTC &= 0b1111110; -> ***************************
			PORTC &= 0b1111100;
			UDR0 = key; // Enviar la tecla presionada

      PORTB |= ~numbersDisplayAnode[number];
      PORTC |= numbersDisplayAnode1[number];

      // ********************************************************

      // Contraseña de 2 dígitos
      valShortPass[count1] = number;
      count1++;
      if (count1 == 2) {
        if ((shortPass[0] == valShortPass[0]) && (shortPass[1] == valShortPass[1])) {
          OCR2A = 100; // Motor con velocidad baja
        }
        count1 = 0;  // Reiniciar el contador
      }

      // Contraseña de 4 dígitos
      valLongPass[count2] = number;
      count2++;

      switch (count2) {
        case 1: PORTC |= (1 << PC2);
        break;
        case 2: PORTC |= (1 << PC3);
        break;
        case 3: PORTC |= (1 << PC4);
        break;
        case 4: PORTC |= (1 << PC5);
        break;
      }

      if (count2 == 4) {
        if ((valLongPass[0]==longPass[0]) && (valLongPass[1]==longPass[1]) && (valLongPass[2]==longPass[2]) && (valLongPass[3]==longPass[3])) {
          OCR2A = 255; // Motor con velocidad máxima
        }
        count2 = 0;
      }
      // ********************************************************
			
			_delay_ms(500);
		}   
	} // End-while
} // End-main

