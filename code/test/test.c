#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL  // Частота работы микроконтроллера 8 МГц

// Определение выводов для сдвигового регистра 74HC595
#define SH_CP_PIN PB0  // Shift register clock (SH_CP)
#define ST_CP_PIN PB2  // Storage register clock (ST_CP)
#define DS_PIN PB1     // Data pin (DS)

// Определение выводов для управления MR и OE
#define MR_PIN PB3     // Подключено к MR
#define OE_PIN PB4     // Подключено к OE

// Определение выводов для управления разрядами индикатора (цифры 1, 2, 3, 4)
#define DIGIT_1 PC5    // Подключено к правому разряду 1
#define DIGIT_2 PC4    // Подключено к правому разряду 2
#define DIGIT_3 PC3    // Подключено к правому разряду 3
#define DIGIT_4 PC2    // Подключено к правому разряду 4

// Определение выводов для отображения режима (частота или амплитуда)
#define FREQ_PIN PB6  // Логическая единица при выводе частоты
#define AMP_PIN PB7   // Логическая единица при выводе амплитуды

// Переменные для частоты и амплитуды
volatile uint16_t signal_frequency = 0;
volatile uint16_t signal_amplitude = 0;
volatile uint16_t adc_value = 0;
volatile uint16_t max_value = 0;
volatile uint16_t min_value = 1023;
volatile uint8_t show_frequency = 1; // 1 - показывать частоту, 0 - амплитуду

// Функция для инициализации UART
void uart_init() {
    // Установить скорость передачи данных 9600 бод
    uint16_t ubrr_value = 51;  // Для частоты 8 МГц и скорости 9600 бод
    UBRR0H = (ubrr_value >> 8);  // Записать старший байт в UBRR0H
    UBRR0L = ubrr_value;         // Записать младший байт в UBRR0L

    // Включить передатчик
    UCSR0B = (1 << TXEN0);  // Включить передатчик (TX)

    // Настроить формат фрейма: 8 бит данных, 1 стоповый бит
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Функция для отправки одного байта через UART
void uart_transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));  // Ожидание готовности передатчика
    UDR0 = data;  // Запись данных в регистр для передачи
}

// Функция для отправки строки через UART с задержкой между символами
void uart_print(const char* str) {
    while (*str) {
        uart_transmit(*str++);  // Отправить каждый символ строки
        _delay_ms(20);  // Задержка для каждого символа
    }
}

// Функция для инициализации АЦП
void adc_init() {
    ADMUX = (1 << REFS0);  // AREF = AVcc, вход ADC0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Включить АЦП, предделитель 64
}

// Функция для чтения значения с АЦП
uint16_t adc_read() {
    ADCSRA |= (1 << ADSC);  // Начать преобразование
    while (ADCSRA & (1 << ADSC));  // Дождаться завершения преобразования
    return ADC;
}

// Функция для отправки байта в сдвиговый регистр 74HC595
void shiftOut(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & (1 << (7 - i))) {
            PORTB |= (1 << DS_PIN);
        } else {
            PORTB &= ~(1 << DS_PIN);
        }
        PORTB |= (1 << SH_CP_PIN);
        _delay_us(10);
        PORTB &= ~(1 << SH_CP_PIN);
    }
    PORTB |= (1 << ST_CP_PIN);
    _delay_us(10);
    PORTB &= ~(1 << ST_CP_PIN);
}

// Функция для инициализации таймера для измерения частоты
void timer1_init() {
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Предделитель на 64
    TCNT1 = 0;  // Сбросить счётчик
}

// Функция для вычисления частоты
void calculate_frequency() {
    static uint16_t last_value = 0;
    static uint16_t zero_cross_time = 0;
    
    // Получить текущее значение АЦП
    adc_value = adc_read();
    
    // Найти пересечение нуля
    if (adc_value >= 512 && last_value < 512) {  // Пересечение с 512 (середина диапазона)
        zero_cross_time = TCNT1;  // Запоминаем время пересечения
        signal_frequency = F_CPU / (64 * zero_cross_time);  // Рассчитать частоту
        TCNT1 = 0;  // Сбросить таймер
    }
    
    last_value = adc_value;
}

// Функция для вычисления амплитуды
void calculate_amplitude() {
    adc_value = adc_read();
    
    // Обновление максимального и минимального значений
    if (adc_value > max_value) {
        max_value = adc_value;
    }
    if (adc_value < min_value) {
        min_value = adc_value;
    }
    
    // Рассчитать амплитуду как разницу между максимумом и минимумом
    signal_amplitude = max_value - min_value;
}

// Инициализация MR и OE
void init_shift_register_control() {
    DDRB |= (1 << MR_PIN) | (1 << OE_PIN);
    PORTB |= (1 << MR_PIN);  // MR = 1
    PORTB &= ~(1 << OE_PIN); // OE = 0
}

// Инициализация управления разрядами индикатора
void init_display_control() {
    DDRC |= (1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_3) | (1 << DIGIT_4);
}

// Инициализация пинов PB6 и PB7 для отображения состояния (частота или амплитуда)
void init_mode_pins() {
    DDRB |= (1 << FREQ_PIN) | (1 << AMP_PIN);  // Настраиваем PB6 и PB7 как выходы
}

// Инициализация кнопки на PD2 с использованием INT0
void button_init() {
    DDRD &= ~(1 << PD2);  // Настроить PD2 как вход
    PORTD |= (1 << PD2);  // Включить подтягивающий резистор для PD2
}

// Инициализация прерывания на INT0 для кнопки на PD2
void int0_init() {
    EICRA |= (1 << ISC01);  // Прерывание по спаду на INT0
    EIMSK |= (1 << INT0);   // Разрешить прерывание INT0
    sei();  // Разрешить глобальные прерывания
}

// Обработчик прерывания на INT0 (кнопка на PD2)
ISR(INT0_vect) {
    show_frequency = !show_frequency;  // Переключение режима отображения
}

// Функция для логирования данных через UART
void log_data() {
    char buffer[32];
    
	uart_print("Frequency: ");
	itoa(signal_frequency, buffer, 10);
	uart_print(buffer);
	uart_print(" Hz\r\n");

	uart_print("Amplitude: ");
	itoa(signal_amplitude, buffer, 10);
	uart_print(buffer);
	uart_print("\r\n\r\n");
    
}

// Функция для переключения между частотой и амплитудой
void set_mode(uint8_t mode) {
    if (mode) {  // Если mode == 1, выводим частоту
        PORTB |= (1 << FREQ_PIN);   // Установить PB6 в 1
        PORTB &= ~(1 << AMP_PIN);   // Установить PB7 в 0
    } else {  // Выводим амплитуду
        PORTB |= (1 << AMP_PIN);    // Установить PB7 в 1
        PORTB &= ~(1 << FREQ_PIN);  // Установить PB6 в 0
    }
}

// Функция для отображения одного разряда (цифры) на индикаторе
void select_digit(uint8_t digit) {
    PORTC &= ~((1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_3) | (1 << DIGIT_4));
    switch (digit) {
        case 1: PORTC |= (1 << DIGIT_1); break;
        case 2: PORTC |= (1 << DIGIT_2); break;
        case 3: PORTC |= (1 << DIGIT_3); break;
        case 4: PORTC |= (1 << DIGIT_4); break;
        default: break;
    }
}

// Массив для отображения цифр на анодном индикаторе
uint8_t digits[10] = {
    0b11000000,  // 0
    0b11111001,  // 1
    0b10100100,  // 2
    0b10110000,  // 3
    0b10011001,  // 4
    0b10010010,  // 5
    0b10000010,  // 6
    0b11111000,  // 7
    0b10000000,  // 8
    0b10010000   // 9
};

// Функция для отображения числа на индикаторе
void display_value(uint16_t value) {
    select_digit(1);
    shiftOut(digits[value % 10]);  // Единицы
    _delay_ms(50);
    select_digit(2);
    shiftOut(digits[(value / 10) % 10]);  // Десятки
    _delay_ms(50);
    select_digit(3);
    shiftOut(digits[(value / 100) % 10]);  // Сотни
    _delay_ms(50);
    select_digit(4);
    shiftOut(digits[(value / 1000) % 10]);  // Тысячи
    _delay_ms(50);
}

int main(void) {
    // Инициализация пинов для работы со сдвиговым регистром
    DDRB |= (1 << SH_CP_PIN) | (1 << ST_CP_PIN) | (1 << DS_PIN);  // Инициализация пинов для сдвигового регистра
    
    // Инициализация систем
    adc_init();
    timer1_init();
    uart_init();  // Инициализация UART
    init_shift_register_control();
    init_display_control();
    init_mode_pins();  // Инициализация пинов PB6 и PB7 для отображения состояния
    button_init();     // Инициализация кнопки на PD2
    int0_init();       // Инициализация прерывания INT0 на PD2
    
    while (1) {
        // Вычисляем частоту и амплитуду
        calculate_frequency();
        calculate_amplitude();
        
        // Отображаем частоту или амплитуду в зависимости от состояния
        if (show_frequency) {
            set_mode(1);  // Установить режим частоты (PB6 = 1, PB7 = 0)
            display_value(signal_frequency);  // Вывод частоты
        } else {
            set_mode(0);  // Установить режим амплитуды (PB7 = 1, PB6 = 0)
            display_value(signal_amplitude);  // Вывод амплитуды
        }

        log_data();  // Логирование данных на виртуальный терминал через UART
        _delay_ms(500);  // Задержка для стабильного обновления
    }
}