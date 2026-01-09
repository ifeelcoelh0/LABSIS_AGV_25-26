#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// ---------------- CONFIG ----------------
#define THRESHOLD       500															     // A baixo de 500 é preto
#define MAX_PWM         200															     // Pwm máximo

#define VEL_BASE		40.0f				//										// RPMS Base

#define K_LINHA			8.0f
#define KD_LINHA		1.5f

#define KP				2.5f				//
#define KI				0.8f				//


// Sensores
const float pesos_sensores[5] = { -4.0f, -2.0f, 0.0f, 2.0f, 4.0f };

// Estados
typedef enum {
	SEGUINDO_LINHA,
	PROCURAR_LINHA
} estado_t;

volatile estado_t estado = SEGUINDO_LINHA;

// Linha
float ultimo_erro = 1.0f;

// Encoders / tempo
volatile uint32_t tempo_ms = 0;
volatile uint16_t pulsos_esq = 0, pulsos_dir = 0;

// timestamps para debounce
volatile uint32_t t_esq = 0;
volatile uint32_t t_dir = 0;


// Referências de velocidade (RPM)
volatile float vel_ref_esq = 0.0f;
volatile float vel_ref_dir = 0.0f;

// Integradores PI
float int_esq = 0.0f;
float int_dir = 0.0f;

// Flag controlo velocidade
volatile uint8_t flag_controle_vel = 0;

// ---------------- ADC ----------------
void init_adc(void)
{
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ler_adc(uint8_t ch)
{
	ADMUX = (ADMUX & 0xF8) | (ch & 0x07) | (1 << REFS0);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

// ---------------- MOTORES (SÓ FRENTE) ----------------
void set_motor_esq(float pwm)
{
	PORTD |= (1 << PD7);
	PORTB &= ~(1 << PB0);

	if (pwm < 0) pwm = 0;
	if (pwm > MAX_PWM) pwm = MAX_PWM;
	OCR0A = (uint8_t)pwm;
}

void set_motor_dir(float pwm)
{
	PORTB |= (1 << PB4);
	PORTB &= ~(1 << PB3);

	if (pwm < 0) pwm = 0;
	if (pwm > MAX_PWM) pwm = MAX_PWM;
	OCR0B = (uint8_t)pwm;
}

// ---------------- ENCODERS (COM DEBOUNCE) ----------------
ISR(INT0_vect)
{
	uint32_t agora = tempo_ms;
	if (agora - t_esq > 2) {      // debounce 2 ms
		pulsos_esq++;
		t_esq = agora;
	}
}

ISR(INT1_vect)
{
	uint32_t agora = tempo_ms;
	if (agora - t_dir > 2) {      // debounce 2 ms
		pulsos_dir++;
		t_dir = agora;
	}
}

// ---------------- TIMER2 – SÓ TIMING ----------------
ISR(TIMER2_COMPA_vect)
{
	tempo_ms += 10;

	static uint8_t c = 0;
	if (++c < 5) return;
	c = 0;

	flag_controle_vel = 1;
}

// ---------------- TIMER1 LED 1Hz ----------------
ISR(TIMER1_OVF_vect)
{
	TCNT1 = 57724;          // 500 ms
	PORTD ^= (1 << PD1);
}

// ---------------- INIT HARDWARE ----------------
void init_hardware(void)
{
	// GPIO
	DDRB |= (1 << PB4) | (1 << PB0) | (1 << PB3);
	DDRD |= (1 << PD5) | (1 << PD6) | (1 << PD1) | (1 << PD7);
	PORTD &= ~(1 << PD1);
	DDRB |= (1 << PB5);
	PORTB &= ~(1 << PB5);

	// PWM Timer0
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);

	// Encoders
	DDRD &= ~((1 << PD2) | (1 << PD3));
	PORTD |= (1 << PD2) | (1 << PD3);
	EICRA |= (1 << ISC00) | (1 << ISC10);
	EIMSK |= (1 << INT0) | (1 << INT1);

	// ADC
	init_adc();

	// Timer2 – controlo motores
	TCCR2A = (1 << WGM21);
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	OCR2A = 155;
	TIMSK2 |= (1 << OCIE2A);

	// Timer1 – LED 1 Hz
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	TCNT1 = 57724;
	TCCR1B |= (1 << CS12) | (1 << CS10);
	TIMSK1 |= (1 << TOIE1);
}

// ---------------- LINHA ----------------
float ler_erro_linha(uint8_t *viu_linha, uint16_t sensores[5])
{
	float soma = 0.0f;
	float cont = 0.0f;
	*viu_linha = 0;

	for (uint8_t i = 0; i < 5; i++) {
		sensores[i] = ler_adc(i);
		if (sensores[i] < THRESHOLD) {
			soma += pesos_sensores[i];
			cont++;
			*viu_linha = 1;
		}
	}

	if (*viu_linha) {
		ultimo_erro = -(soma / cont);
	}

	// Atualiza LED de debug PB5
	if (*viu_linha)
	PORTB |= (1 << PB5);
	else
	PORTB &= ~(1 << PB5);

	return ultimo_erro;
}

// ---------------- CONTROLO ----------------
void controlo_linha(uint16_t sensores[5])
{
	uint8_t viu_linha;
	ler_erro_linha(&viu_linha, sensores);

	static estado_t estado_antigo = SEGUINDO_LINHA;
	static float erro_anterior = 0.0f;

	if (viu_linha)
	estado = SEGUINDO_LINHA;
	else
	estado = PROCURAR_LINHA;

	// -------- RESET DO INTEGRADOR AO MUDAR DE ESTADO --------
	if (estado_antigo != estado) {
		if (estado == SEGUINDO_LINHA) {
			int_esq = 0.0f;
			int_dir = 0.0f;
			erro_anterior = ultimo_erro;
		}
		estado_antigo = estado;
	}

	if (estado == SEGUINDO_LINHA) {
		
		
		float dt = 0.01f;   // tempo entre chamadas do controlo_linha (ajusta se preciso)

		float erro = ultimo_erro;
		float derivada = (erro - erro_anterior) / dt;

		float delta = K_LINHA * erro + KD_LINHA * derivada;

		erro_anterior = erro;

		vel_ref_esq = VEL_BASE - delta;
		vel_ref_dir = VEL_BASE + delta;
		
		if (vel_ref_esq < 0) vel_ref_esq = 0;
		if (vel_ref_dir < 0) vel_ref_dir = 0;

		if (vel_ref_esq > 80) vel_ref_esq = 80;
		if (vel_ref_dir > 80) vel_ref_dir = 80;
		
		} else {
		if (ultimo_erro > 0) {
			vel_ref_esq = 0;
			vel_ref_dir = VEL_BASE*1.2;
			} else {
			vel_ref_esq = VEL_BASE*1.2;
			vel_ref_dir = 0;
		}
	}
}

void controlo_velocidade(void)
{
	float dt = 0.05;

	float rpm_esq = pulsos_esq * (60.0f / (40.0f * dt));
	float rpm_dir = pulsos_dir * (60.0f / (40.0f * dt));
	pulsos_esq = pulsos_dir = 0;

	float erro_esq = vel_ref_esq - rpm_esq;
	float erro_dir = vel_ref_dir - rpm_dir;

	int_esq += erro_esq * dt;
	int_dir += erro_dir * dt;

	if (int_esq > 250)  int_esq =  250;
	if (int_esq < -250) int_esq = -250;
	if (int_dir > 250)  int_dir =  250;
	if (int_dir < -250) int_dir = -250;

	float out_esq = KP * erro_esq + KI * int_esq;
	float out_dir = KP * erro_dir + KI * int_dir;

	if (out_esq > MAX_PWM) out_esq = MAX_PWM;
	if (out_esq < 0)       out_esq = 0;

	if (out_dir > MAX_PWM) out_dir = MAX_PWM;
	if (out_dir < 0)       out_dir = 0;

	
	set_motor_esq(out_esq);
	set_motor_dir(out_dir);
	
}

// ---------------- MAIN ----------------
int main(void)
{
	init_hardware();
	sei();

	uint16_t sensores[5];

	while (1) {
		// -------- CONTROLO DA LINHA --------
		controlo_linha(sensores);

		// -------- CONTROLO DE VELOCIDADE --------
		if (flag_controle_vel) {
			flag_controle_vel = 0;
			controlo_velocidade();
		}
	}
}